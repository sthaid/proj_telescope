/*
Copyright (c) 2018 Steven Haid

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "common.h"
#include <util_sky.h>

//
// defines
//

#define SKY_TIME_MODE_NONE      0
#define SKY_TIME_MODE_CURRENT   1
#define SKY_TIME_MODE_PAUSED    2
#define SKY_TIME_MODE_REV       3
#define SKY_TIME_MODE_FWD       4
#define SKY_TIME_MODE_REV_STEP  5
#define SKY_TIME_MODE_FWD_STEP  6

#define SKY_TIME_STEP_MODE_NONE      0
#define SKY_TIME_STEP_MODE_DELTA_T   1
#define SKY_TIME_STEP_MODE_SUNRISE   2
#define SKY_TIME_STEP_MODE_SUNSET    3
#define SKY_TIME_STEP_MODE_SIDDAY    4
#define SKY_TIME_STEP_MODE_TIMEOFDAY 5

#define SKY_TIME_MODE_STR(x) \
    ({ int _mode = (x); \
       (_mode == SKY_TIME_MODE_NONE      ? "NONE"     : \
        _mode == SKY_TIME_MODE_CURRENT   ? "CURR"     : \
        _mode == SKY_TIME_MODE_PAUSED    ? "PAUSE"    : \
        _mode == SKY_TIME_MODE_FWD       ? "FWD"      : \
        _mode == SKY_TIME_MODE_REV       ? "REV"      : \
        _mode == SKY_TIME_MODE_FWD_STEP  ? "FWD_STEP" : \
        _mode == SKY_TIME_MODE_REV_STEP  ? "REV_STEP" : \
                                           "???"  );})

#define SKY_TIME_STEP_MODE_STR(x) \
    ({ int _step_mode = (x); \
       (_step_mode == SKY_TIME_STEP_MODE_NONE      ? "NONE"      : \
        _step_mode == SKY_TIME_STEP_MODE_DELTA_T   ? "DELTA_T"   : \
        _step_mode == SKY_TIME_STEP_MODE_SUNRISE   ? "SUNRISE"   : \
        _step_mode == SKY_TIME_STEP_MODE_SUNSET    ? "SUNSET"    : \
        _step_mode == SKY_TIME_STEP_MODE_SIDDAY    ? "SIDDAY"    : \
        _step_mode == SKY_TIME_STEP_MODE_TIMEOFDAY ? "TIMEOFDAY" : \
                                                     "???"  );})

#define DEG2RAD (M_PI / 180.)

#define MIN_MAG     -5    // brightest
#define MAX_MAG      22   // dimmest
#define DEFAULT_MAG  7    // faintest naked-eye stars from dark rural area

#define SID_DAY_SECS  (23*3600 + 56*60 + 4)

#define DEFAULT_SKY_TIME_STEP_MODE_DELTA_T 0.1  // hours

#define TRACKING_OFF   -1
#define TRACKING_RADEC -2
#define IDENT_OFF      -1

//
// typedefs
//

typedef struct {
    char *name;
    int type;
    double ra;
    double dec;
    double mag;
    double az;
    double el;
    int x;
    int y;
    int xvp;
    int yvp;
} obj_t;

//
// variables
//

static obj_t *obj          = NULL;
static time_t sky_time     = 0;
static double lst          = 0;
static double az_ctr       = 0;   // range -180 to +180
static double az_span      = 360;
static double el_ctr       = 45;
static double el_span      = 90;  
static double mag          = DEFAULT_MAG;
static int    tracking     = TRACKING_OFF;
static double tracking_ra  = 0;
static double tracking_dec = 0;
static int    ident        = IDENT_OFF;

//
// prototypes
//

// pane support 
static char *trk_str(void);
static char *ident_str(void);
static char *sky_pane_cmd(char * cmd_line);
static int azel2xy(double az, double el, double max, double *xret, double *yret);

// sky time utils
static void sky_time_set_mode(int mode);
static void sky_time_get_mode(int * mode);
static void sky_time_set_step_mode(int step_mode, double step_mode_hr_param);
static void sky_time_get_step_mode(int *step_mode, double *step_mode_hr_param);
static time_t sky_time_get_time(void);
static time_t sky_time_tod_next(time_t t, double hr);
static time_t sky_time_tod_prior(time_t t, double hr);

// general utils
static void reset(bool all_sky);
static void get_next_day(int * year, int * month, int * day);
static void get_prior_day(int * year, int * month, int * day);

// -----------------  SKY INIT  -------------------------------------------

int sky_init(char *incl_obj_str)
{
    int ret;

    ret = util_sky_init(incl_obj_str, false, true);
    if (ret < 0) {
        ERROR("util_sky_init failed\n");
        return ret;
    }

    obj = calloc(max_obj, sizeof(obj_t));
    if (obj == NULL) {
        ERROR("failed calloc max_obj=%d sizeof(obj_t)=%ld\n", max_obj, sizeof(obj_t));
        return -1;
    }

    return 0;
}

// -----------------  SKY PANE HANDLER  -----------------------------------

int sky_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event) 
{
    struct {
        char   cmd_line[1000];
        char   cmd_line_last[1000];
        int    cmd_line_len;
        char * cmd_status;
        long   cmd_status_time_us;
    } * vars = pane_cx->vars;
    rect_t * pane = &pane_cx->pane;

    #define SDL_EVENT_MOUSE_MOTION   (SDL_EVENT_USER_DEFINED + 0)
    #define SDL_EVENT_MOUSE_WHEEL    (SDL_EVENT_USER_DEFINED + 1)
    #define SDL_EVENT_MOUSE_RIGHT_CLICK    (SDL_EVENT_USER_DEFINED + 2)

    // ----------------------------
    // -------- INITIALIZE --------
    // ----------------------------

    if (request == PANE_HANDLER_REQ_INITIALIZE) {
        vars = pane_cx->vars = calloc(1,sizeof(*vars));
        DEBUG("PANE x,y,w,h  %d %d %d %d\n",
              pane->x, pane->y, pane->w, pane->h);
        return PANE_HANDLER_RET_NO_ACTION;
    }

    // ------------------------
    // -------- RENDER --------
    // ------------------------

    if (request == PANE_HANDLER_REQ_RENDER) {
        double min_az, max_az, min_el, max_el, k_el, k_az;
        int xcoord, ycoord, i, ptsz, color, fontsz, row, mode, step_mode, ret;;
        double az, el;
        double grid_sep, first_az_line, first_el_line;
        double step_mode_hr_param;
        char str[100], time_mode_str[100], *s;

        // get sky_time, which can be one of the following:
        // - current time
        // - fast time
        // - advance a siderial day at a time
        // - pause
        // and get local siderial time
        sky_time = sky_time_get_time();
        lst = ct2lst(longitude, jdconv2(sky_time));

        // if we're tracking then update the az/el of sky_pane center to the
        // az/el of the tracking object
        if (tracking >= 0) {
            obj_t x;
            get_obj(tracking, sky_time, lst, &x.name, &x.type, &x.ra, &x.dec, &x.mag, &x.az, &x.el);
            radec2azel(&az_ctr, &el_ctr, x.ra, x.dec, lst, latitude);
            if (az_ctr > 180) az_ctr -= 360;
        } else if (tracking == TRACKING_RADEC) {
            radec2azel(&az_ctr, &el_ctr, tracking_ra, tracking_dec, lst, latitude);
            if (az_ctr > 180) az_ctr -= 360;
        }

        // init variables that are dependant on az_ctr/el_ctr, which
        // were just determined above
        min_az = az_ctr - az_span / 2.;
        max_az = az_ctr + az_span / 2.;
        min_el = el_ctr - el_span / 2.;
        max_el = el_ctr + el_span / 2.;
        k_el = (pane->h) / (max_el - min_el);
        k_az = (pane->w) / (min_az - max_az);

        // draw points for objects
        for (i = 0; i < max_obj; i++) {
            obj_t *x = &obj[i];

            // get info for object 'i', and
            // reset other obj_t fields to NO_VALUE
            // note: get_obj can fail if the sky_time is out of range for a solar-sys object
            ret = get_obj(i, sky_time, lst, &x->name, &x->type, &x->ra, &x->dec, &x->mag, &x->az, &x->el);
            if (ret != 0) {
                continue;
            }
            x->x   = NO_VALUE;
            x->y   = NO_VALUE;
            x->xvp = NO_VALUE;
            x->yvp = NO_VALUE;

            // if obj type is not valid then fail
            if (x->type == OBJTYPE_NONE) {
                FATAL("BUG: obj %d does not exist\n", i);
            }

            // magnitude too dim then skip; 
            // however don't skip when magnitude is NO_VALUE, such as
            // for place marks or when a solar sys obj has 'n.a.' magnitude
            if (x->mag != NO_VALUE && x->mag > mag) {
                continue;
            }

            // The azimuth returned from get_obj is in range 0 to 360; 
            // however the min_az..max_az could be as low as -360 to 0;
            // so, if az is too large, try to correct by reducing it by 360.
            if (x->az > max_az) {
                x->az -= 360;
            }

            // if az or el out of range then skip
            if (x->az < min_az || x->az > max_az || x->el < min_el || x->el > max_el) {
                continue;
            }

            // determine display point size from object's apparent magnitude
            ptsz = (x->mag != NO_VALUE ? 5 - x->mag : 3);
            if (ptsz < 0) ptsz = 0;
            if (ptsz > 9) ptsz = 9;

            // convert az/el to pane coordinates; 
            // if coords are out of the pane then skip
            ycoord = 0 + k_el * (max_el - x->el);
            xcoord = 0 + k_az * (min_az - x->az);
            if (xcoord < 0 || xcoord >= pane->w || ycoord < 0 || ycoord >= pane->h) {
                continue;
            }

            // save the computed object's pane coords so that they can be used
            // when processing the right-click event, to find the object that is
            // nearest the mouse's right-click x,y pane coordinates
            x->x = xcoord;
            x->y = ycoord;

            // render the stellar object point
            color = (i == tracking                 ? RED    :
                     i == ident                    ? ORANGE :
                     x->type == OBJTYPE_STELLAR    ? WHITE  :
                     x->type == OBJTYPE_SOLAR      ? YELLOW :
                     x->type == OBJTYPE_PLACE_MARK ? PURPLE :
                                                     BLACK);
            if (color == BLACK) {
                FATAL("BUG: obj %d '%s' invalid type %d\n", i, x->name, x->type);
            }
            sdl_render_point(pane, xcoord, ycoord, color, ptsz);
        }

        // draw grid
        grid_sep = az_span > 240 ? 45 :
                   az_span > 120 ? 20 :
                   az_span > 60  ? 10 :
                   az_span > 25  ? 5 :
                   az_span > 10  ? 2 :
                   az_span > 6   ? 1 :
                                   0.5;
        fontsz = 22;  // grid
        first_az_line = floor(min_az/grid_sep) * grid_sep;
        for (az = first_az_line; az <= max_az; az += grid_sep) {
            double adj_az = (az >= 0 ? az : az + 360);
            int len = sprintf(str, "%g", adj_az);
            xcoord = 0 + k_az * (min_az - az);
            sdl_render_line(pane, xcoord, 0, xcoord, pane->h-1, BLUE);
            if (xcoord < COL2X(len+2,fontsz)) {
                continue;
            }
            sdl_render_printf(pane, xcoord-COL2X(len,fontsz)/2, pane->h-ROW2Y(1,fontsz), fontsz, WHITE, BLACK, "%g", adj_az);
        }
        first_el_line = floor(min_el/grid_sep) * grid_sep;
        for (el = first_el_line; el <= max_el; el += grid_sep) {
            ycoord = 0 + k_el * (max_el - el);
            sdl_render_line(pane, 0, ycoord, pane->w-1, ycoord, BLUE);
            if (ycoord > pane->h - ROW2Y(1,fontsz)/2 && 
                ycoord < pane->h + ROW2Y(1,fontsz)/4) 
            {
                ycoord = pane->h - ROW2Y(1,fontsz)/2;
            }
            if (ycoord > -ROW2Y(1,fontsz)/4 && 
                ycoord < ROW2Y(1,fontsz)/2) 
            {
                ycoord = ROW2Y(1,fontsz)/2;
            }
            sdl_render_printf(pane, 0, ycoord-ROW2Y(1,fontsz)/2, fontsz, BLUE, BLACK, "%g", el);
        }

        // draw cross hair
        sdl_render_line(pane, 
                        pane->w/2-20, pane->h/2, pane->w/2+20, pane->h/2, 
                        RED);
        sdl_render_line(pane, 
                        pane->w/2, pane->h/2-20, pane->w/2, pane->h/2+20, 
                        RED);

        // display names of the objects that have names
        fontsz = 18;  // objects
        for (i = 0; i < max_obj; i++) {
            obj_t * x = &obj[i];
            if (x->name[0] != '\0' && 
                x->x != NO_VALUE &&
                (x->type == OBJTYPE_SOLAR || az_span < 100))
            {
                sdl_render_printf(pane, 
                                x->x+6, x->y-ROW2Y(1,fontsz)/2,
                                fontsz, WHITE, BLACK, "%s", x->name);
            }
        }

        // display the date/time, az and el of center, and magnitude;
        // also display time mode if it is not SKY_TIME_MODE_CURRENT
        fontsz = 22;  // date/time line
        row = 0;
        sky_time_get_mode(&mode);
        sky_time_get_step_mode(&step_mode, &step_mode_hr_param);
        if (mode != SKY_TIME_MODE_CURRENT) {
            sprintf(time_mode_str, "%s", SKY_TIME_STEP_MODE_STR(step_mode));
            if (step_mode == SKY_TIME_STEP_MODE_TIMEOFDAY ||
                step_mode == SKY_TIME_STEP_MODE_DELTA_T) 
            {
                sprintf(time_mode_str+strlen(time_mode_str), ":%g", step_mode_hr_param);
            }
            sprintf(time_mode_str+strlen(time_mode_str), ":%s", SKY_TIME_MODE_STR(mode));
        } else {
            time_mode_str[0] = '\0';
        }
        color = (mode == SKY_TIME_MODE_CURRENT ? WHITE :
                 mode == SKY_TIME_MODE_PAUSED  ? RED   :
                                                 GREEN);
        sdl_render_printf(pane, COL2X(3,fontsz), ROW2Y(row,fontsz), fontsz, color, BLACK, 
                          "%s  AZ=%-5.2f  EL=%-5.2f  MAG=%0.1f  %s",
                          localtime_str(sky_time,str), 
                          (az_ctr < 0 ? az_ctr + 360 : az_ctr), el_ctr, 
                          mag, time_mode_str);
        row++;

        // display tracking string, if present
        s = trk_str();
        if (s[0]) {
            sdl_render_printf(pane, COL2X(3,fontsz), ROW2Y(row,fontsz), fontsz, WHITE, BLACK, "%s", s);
            row++;
        }

        // display ident string, if present
        s = ident_str();
        if (s[0]) {
            sdl_render_printf(pane, COL2X(3,fontsz), ROW2Y(row,fontsz), fontsz, WHITE, BLACK, "%s", s);
            row++;
        }

        // display the cmd_line and cmd_status
        if (microsec_timer() - vars->cmd_status_time_us < 2500000) {
            sdl_render_printf(pane, COL2X(3,fontsz), ROW2Y(row,fontsz), fontsz, WHITE, BLACK,
                              "> %s", vars->cmd_line_last);
            sdl_render_printf(pane, COL2X(3,fontsz), ROW2Y(row+1,fontsz), fontsz, WHITE, BLACK,
                              "  %s", vars->cmd_status);
        } else {
            sdl_render_printf(pane, COL2X(3,fontsz), ROW2Y(row,fontsz), fontsz, WHITE, BLACK,
                              "> %s%c", vars->cmd_line, '_');
        }

        // clear tracking/ident if the magnitude of the selected obj is no longer being displayed
        if (tracking >= 0 && obj[tracking].mag != NO_VALUE && obj[tracking].mag > mag) {
            tracking = TRACKING_OFF;
        }
        if (ident >= 0 && obj[ident].mag != NO_VALUE && obj[ident].mag > mag) {
            ident = IDENT_OFF;
        }

        // register control events 
        rect_t loc = {0,0,pane->w,pane->h};
        sdl_register_event(pane, &loc, SDL_EVENT_MOUSE_MOTION, SDL_EVENT_TYPE_MOUSE_MOTION, pane_cx);
        sdl_register_event(pane, &loc, SDL_EVENT_MOUSE_WHEEL, SDL_EVENT_TYPE_MOUSE_WHEEL, pane_cx);
        sdl_register_event(pane, &loc, SDL_EVENT_MOUSE_RIGHT_CLICK, SDL_EVENT_TYPE_MOUSE_RIGHT_CLICK, pane_cx);

        //return ret_action;;
        return PANE_HANDLER_RET_NO_ACTION;
    }

    // -----------------------
    // -------- EVENT --------
    // -----------------------

    if (request == PANE_HANDLER_REQ_EVENT) {
        switch (event->event_id) {
        case SDL_EVENT_MOUSE_MOTION: 
        case SDL_EVENT_KEY_UP_ARROW: 
        case SDL_EVENT_KEY_DOWN_ARROW: 
        case SDL_EVENT_KEY_LEFT_ARROW: 
        case SDL_EVENT_KEY_RIGHT_ARROW: {
            int dx=0, dy=0;

            if (event->event_id == SDL_EVENT_MOUSE_MOTION) {
                dx = event->mouse_motion.delta_x;
                dy = event->mouse_motion.delta_y;
            } else if (event->event_id == SDL_EVENT_KEY_LEFT_ARROW) {
                dx = -5;
            } else if (event->event_id == SDL_EVENT_KEY_RIGHT_ARROW) {
                dx = +5;
            } else if (event->event_id == SDL_EVENT_KEY_UP_ARROW) {
                dy = -5;
            } else if (event->event_id == SDL_EVENT_KEY_DOWN_ARROW) {
                dy = +5;
            }

            if (dx == 0 && dy == 0) {
                break;
            }

            az_ctr -= dx * (az_span / 1800.);
            if (az_ctr >  180) az_ctr -= 360;
            if (az_ctr < -180) az_ctr += 360;

            el_ctr += dy * (el_span / 450.);
            if (el_ctr > 90) el_ctr = 90;
            if (el_ctr < -90) el_ctr = -90;

            if (tracking != TRACKING_OFF) {
                tracking = TRACKING_RADEC;
                azel2radec(&tracking_ra, &tracking_dec, az_ctr, el_ctr, lst, latitude);
            }

            DEBUG("MOUSE MOTION dx=%d dy=%d  az_ctr=%f el_ctr=%f\n", dx, dy, az_ctr, el_ctr);
            return PANE_HANDLER_RET_DISPLAY_REDRAW; }

        case SDL_EVENT_MOUSE_WHEEL: {
            int dy = event->mouse_motion.delta_y;
            if (dy < 0 && az_span < 359.99) {
                az_span *= 1.1;
                el_span *= 1.1;
            }
            if (dy > 0 && az_span > 3) {
                az_span /= 1.1;
                el_span /= 1.1;
            }
            DEBUG("MOUSE WHEEL dy=%d  az_span=%f el_span=%f\n", dy, az_span, el_span);
            return PANE_HANDLER_RET_DISPLAY_REDRAW; }

        case SDL_EVENT_MOUSE_RIGHT_CLICK: {
            int mouse_x, mouse_y;
            int dist, best_dist, best_i, i, delta_x, delta_y;

            static int distance_lookup[50][50];
            static bool first_call = true;

            if (first_call) {
                int x, y;
                for (x = 0; x < 50; x++) {
                    for (y = 0; y < 50; y++) {
                        distance_lookup[x][y] = sqrt(x*x + y*y);
                    }
                }
                first_call = false;
            }

            mouse_x = event->mouse_click.x;
            mouse_y = event->mouse_click.y;
            best_dist = 999999;
            best_i = -1;
            
            for (i = 0; i < max_obj; i++) {
                obj_t * x = &obj[i];
                if (x->x == NO_VALUE) {
                    continue;
                }
                delta_x = abs(x->x - mouse_x);
                delta_y = abs(x->y - mouse_y);
                if (delta_x >= 50 || delta_y >= 50) {
                    continue;
                }
                dist = distance_lookup[delta_x][delta_y];
                if (dist < best_dist) {
                    best_dist = dist;
                    best_i = i;
                }
            }
            ident = (best_dist != 999999 ? best_i : IDENT_OFF);
            return PANE_HANDLER_RET_DISPLAY_REDRAW; }

        case 1 ... 127: {
            char ch = event->event_id;
            if (ch >= 32 && ch <= 126) {
                vars->cmd_line[vars->cmd_line_len++] = ch;
                vars->cmd_line[vars->cmd_line_len] = '\0';
            }
            if (ch == '\b') { // backspace
                if (vars->cmd_line_len > 0) {
                    vars->cmd_line_len--;
                    vars->cmd_line[vars->cmd_line_len] = '\0';
                }
            }
            if (ch == '\r') { // carriage return
                if (vars->cmd_line_len > 0) {
                    vars->cmd_status = sky_pane_cmd(vars->cmd_line);
                    vars->cmd_status_time_us = microsec_timer();
                    strcpy(vars->cmd_line_last, vars->cmd_line);
                    vars->cmd_line_len = 0;
                    vars->cmd_line[0]  = '\0';
                }
            }
            return PANE_HANDLER_RET_DISPLAY_REDRAW; }

        case SDL_EVENT_KEY_ALT + 'm': 
        case SDL_EVENT_KEY_ALT + 'M':
            mag += (event->event_id == SDL_EVENT_KEY_ALT + 'M' ? .1 : -.1);
            if (mag < MIN_MAG) mag = MIN_MAG;
            if (mag > MAX_MAG) mag = MAX_MAG;
            return PANE_HANDLER_RET_DISPLAY_REDRAW;

        case SDL_EVENT_KEY_ALT + '1':
            sky_time_set_mode(SKY_TIME_MODE_CURRENT);
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_ALT + '2':
            sky_time_set_mode(SKY_TIME_MODE_PAUSED);
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_ALT + '3':
            sky_time_set_mode(SKY_TIME_MODE_REV);
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_ALT + '4':
            sky_time_set_mode(SKY_TIME_MODE_FWD);
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_ALT + '5':
            sky_time_set_mode(SKY_TIME_MODE_REV_STEP);
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_ALT + '6':
            sky_time_set_mode(SKY_TIME_MODE_FWD_STEP);
            return PANE_HANDLER_RET_DISPLAY_REDRAW;

        case SDL_EVENT_KEY_PGUP:
            reset(false);
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_PGDN:
            reset(true);
            return PANE_HANDLER_RET_DISPLAY_REDRAW;

        case SDL_EVENT_KEY_HOME:
            az_ctr = az_cal_pos;
            el_ctr = el_cal_pos;
            tracking = TRACKING_OFF;
            ident = IDENT_OFF;
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        }

        return PANE_HANDLER_RET_NO_ACTION;
    }

    // ---------------------------
    // -------- TERMINATE --------
    // ---------------------------

    if (request == PANE_HANDLER_REQ_TERMINATE) {
        free(vars);
        return PANE_HANDLER_RET_NO_ACTION;
    }

    // not reached
    assert(0);
    return PANE_HANDLER_RET_NO_ACTION;
}

static char * sky_pane_cmd(char * cmd_line)
{
    int i;
    char *cmd, *arg1;
    char cmd_line_copy[1000];

    // tokenize cmd_line into cmd, arg1
    strcpy(cmd_line_copy, cmd_line);
    cmd = strtok(cmd_line_copy, " ");
    arg1 = strtok(NULL, "");
    INFO("cmd='%s' arg1='%s'\n", cmd, arg1);

    // ignore blank cmd_line
    if (cmd == NULL) {
        return "";
    }

    // cmd: trk <[objname]|ident|curr|off>
    if (strcasecmp(cmd, "trk") == 0) {
        if (arg1 == NULL) {
            return "error: arg expected";
        }

        if (strcasecmp(arg1, "ident") == 0) {
            if (ident == IDENT_OFF) {
                tracking = TRACKING_OFF;
                return "error: no ident";
            } else {
                tracking = ident;
                ident = IDENT_OFF;
                return "okay";
            }
        } else if (strcasecmp(arg1, "curr") == 0) {
            tracking = TRACKING_RADEC;
            azel2radec(&tracking_ra, &tracking_dec, az_ctr, el_ctr, lst, latitude);
            return "okay";
        } else if (strcasecmp(arg1, "off") == 0) {
            tracking = TRACKING_OFF;
            return "okay";
        } else {
            for (i = 0; i < max_obj; i++) {
                obj_t *x = &obj[i];
                if (strcasecmp(arg1, x->name) == 0) {
                    if (x->mag != NO_VALUE && x->mag > mag) {
                        return "error: obj too dim";
                    }
                    tracking = i;
                    return "okay";
                }
            }
            tracking = TRACKING_OFF;
            return "error: not found";
        }
        FATAL("BUG: shouldn't be here\n");
    }

    // cmd: ident <[objname]|off>
    if (strcasecmp(cmd, "ident") == 0) {
        if (arg1 == NULL) {
            return "error: arg expected";
        }

        if (strcasecmp(arg1, "off") == 0) {
            ident = IDENT_OFF;
            return "okay";
        } else {
            for (i = 0; i < max_obj; i++) {
                obj_t *x = &obj[i];
                if (strcasecmp(arg1, x->name) == 0) {
                    if (x->mag != NO_VALUE && x->mag > mag) {
                        return "error: obj too dim";
                    }
                    ident = i;
                    return "okay";
                }
            }
            ident = IDENT_OFF;
            return "error: not found";
        }
        FATAL("BUG: shouldn't be here\n");
    }

    // cmd: tstep <delta_t|sunrise[+/-h.hh]|sunset[+/-h.hh]|sidday|h.hhh>
    if (strcasecmp(cmd, "tstep") == 0) {
        // This command sets the time-mode that is used when SKY_TIME_MODE_REV/FWD
        // is enabled. SKY_TIME_MODE_xxx is controlled using the key cmds 1,2,3,4,5,6
        // in the sky pane.
        // 
        // EXAMPLES           TIME OF NEXT DISPLAY UPDATE IN SKY_TIME_MODE_FWD
        // --------           ------------------------------------------------
        // tstep delta_t      current time + .1 hours  (the default)
        // tstep delta_t=1    current time + 1 hours
        // tstep sunset       next day sunset
        // tstep sunset+2.5   next day sunset + 2 1/2 hours
        // tstep sidday       current time + SID_DAY_SECS
        // tstep 23.25        23:15:00 Localtime of the next day 

        double hr = 0;
        if (arg1 == NULL) {
            return "error: expected <delta_t[=hr]|sidday|sunset[+/-hr]|sunrise[+/-hr]|hr>";
        }

        if (strncasecmp(arg1, "delta_t", 7) == 0) {
            hr = DEFAULT_SKY_TIME_STEP_MODE_DELTA_T;
            if (arg1[7] == '=') {
                if (sscanf(arg1+8, "%lf", &hr) != 1) {
                    return "error: invalid delta_t hr";
                }
            } else if (arg1[7] != '\0') {
                return "error: invalid delta_t hr";
            }
            sky_time_set_step_mode(SKY_TIME_STEP_MODE_DELTA_T, hr);
            return "okay";
        } else if (strcasecmp(arg1, "sidday") == 0) {
            sky_time_set_step_mode(SKY_TIME_STEP_MODE_SIDDAY, 0);
            return "okay";
        } else if (strncasecmp(arg1, "sunset", 6) == 0) {
            if (arg1[6] == '+' || arg1[6] == '-') {
                if (sscanf(arg1+6, "%lf", &hr) != 1) {
                    return "error: invalid hour";
                }
            } else if (arg1[6] != '\0') {
                return "error: invalid hour";
            }
            sky_time_set_step_mode(SKY_TIME_STEP_MODE_SUNSET, hr);
            return "okay";
        } else if (strncasecmp(arg1, "sunrise", 7) == 0) {
            if (arg1[7] == '+' || arg1[7] == '-') {
                if (sscanf(arg1+7, "%lf", &hr) != 1) {
                    return "error: invalid hour";
                }
            } else if (arg1[7] != '\0') {
                return "error: invalid hour";
            }
            sky_time_set_step_mode(SKY_TIME_STEP_MODE_SUNRISE, hr);
            return "okay";
        } else if (sscanf(arg1, "%lf", &hr) == 1) {
            if (hr < 0 || hr >= 24) {
                return "error: invalid hour";
            }
            sky_time_set_step_mode(SKY_TIME_STEP_MODE_TIMEOFDAY, hr);
            return "okay";
        } else {
            return "error: invalid arg";
        }
    }

    // cmd: quit
    if (strcasecmp(cmd, "quit") == 0) {
        exit(0);
        // not reached
    }

    // invalid command
    return "error: invalid command";
}

static char * trk_str(void)
{
    static char str[200];

    switch (tracking) {
    case TRACKING_OFF:
        // not tracking
        str[0] = '\0';
        break;
    case TRACKING_RADEC:
        // tracking ra/dec
        sprintf(str, "TRK   %0.2f %0.2f", tracking_ra, tracking_dec);
        break;
    default: {
        // tracking an object
        obj_t *x = &obj[tracking];
        char name_str[100], mag_str[100];
        double az, el;

        if (x->name[0] != '\0') {
            strcpy(name_str, x->name);
        } else {
            sprintf(name_str, "Object-%d", tracking);
        }

        if (x->mag != NO_VALUE) {
            sprintf(mag_str, "%0.1f", x->mag);
        } else {
            mag_str[0] = '\0';
        }

        radec2azel(&az, &el, x->ra, x->dec, lst, latitude);

        sprintf(str, "TRK   %s %0.2f %0.2f %s : %0.2f %0.2f",
                name_str, x->ra, x->dec, mag_str, az, el);
        break; }
    }

    return str;
}

static char * ident_str(void)
{
    static char str[200];

    switch (ident) {
    case IDENT_OFF:
        str[0] = '\0';
        break;
    default: {
        // ident an object
        obj_t *x = &obj[ident];
        char name_str[100], mag_str[100];
        double az, el;

        if (x->name[0] != '\0') {
            strcpy(name_str, x->name);
        } else {
            sprintf(name_str, "Object-%d", ident);
        }

        if (x->mag != NO_VALUE) {
            sprintf(mag_str, "%0.1f", x->mag);
        } else {
            mag_str[0] = '\0';
        }

        radec2azel(&az, &el, x->ra, x->dec, lst, latitude);

        sprintf(str, "IDENT %s %0.2f %0.2f %s : %0.2f %0.2f",
                name_str, x->ra, x->dec, mag_str, az, el);
        break; }
    }

    return str;
}

// returns the az/el that the telescope should be pointed
void sky_get_tgt_azel(double * tgt_az, double * tgt_el)
{
    *tgt_az = az_ctr;
    *tgt_el = el_ctr;
}

// -----------------  SKY VIEW PANE HANDLER  ------------------------------

static int    sky_view_scale_tbl_idx = 0;
static double sky_view_scale_tbl[]   = {45, 40, 35, 30, 25, 20, 15, 10, 5, 4, 3, 2, 1, 0.5};

int sky_view_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event) 
{
    struct {
        int tbd;
    } * vars = pane_cx->vars;
    rect_t * pane = &pane_cx->pane;

    #define MAX_SCALE_TBL (sizeof(sky_view_scale_tbl)/sizeof(sky_view_scale_tbl[0]))

    #define SDL_EVENT_MOUSE_MOTION   (SDL_EVENT_USER_DEFINED + 0)
    #define SDL_EVENT_MOUSE_WHEEL    (SDL_EVENT_USER_DEFINED + 1)

    // ----------------------------
    // -------- INITIALIZE --------
    // ----------------------------

    if (request == PANE_HANDLER_REQ_INITIALIZE) {
        vars = pane_cx->vars = calloc(1,sizeof(*vars));
        DEBUG("PANE x,y,w,h  %d %d %d %d\n",
            pane->x, pane->y, pane->w, pane->h);
        return PANE_HANDLER_RET_NO_ACTION;
    }

    // ------------------------
    // -------- RENDER --------
    // ------------------------

    if (request == PANE_HANDLER_REQ_RENDER) {
        int i, ret, ptsz, color, xcoord, ycoord, fontsz;
        double xret, yret, max;

        // display objects that fall within the sky view pane
        for (i = 0; i < max_obj; i++) {
            obj_t * x = &obj[i];

            // if obj type is not valid then continue
            if (x->type == OBJTYPE_NONE) {
                continue;
            }

            // magnitude too dim then skip;
            // however don't skip when magnitude is NO_VALUE, such as
            // for place marks or when a solar sys obj has 'n.a.' magnitude
            if (x->mag != NO_VALUE && x->mag > mag) {
                continue;
            }

            // if az/el is not available for this object then skip
            if (x->az == NO_VALUE || x->el == NO_VALUE) {
                continue;
            }

            // determine the minimally distorted position of the object
            // relative to az_ctr/el_ctr;
            // if the angle from az_ctr/el_ctr to az/el exceeds max the skip this object
            max = sky_view_scale_tbl[sky_view_scale_tbl_idx];
            ret = azel2xy(x->az, x->el, max, &xret, &yret);
            if (ret != 0) {
                continue;
            }

            // determine display point size from object's apparent magnitude
            ptsz = (x->mag != NO_VALUE ? 5 - x->mag : 3);
            if (ptsz < 0) ptsz = 0;
            if (ptsz > 9) ptsz = 9;

            // convert xret/yret to pane coordinates;
            // if coords are out of the pane then skip
            xcoord = pane->w/2 + xret * pane->w/2;
            ycoord = pane->h/2 - yret * pane->h/2;
            if (xcoord < 0 || xcoord >= pane->w || ycoord < 0 || ycoord >= pane->h) {
                continue;
            }

            // save the computed object's pane coords so that they can be used below
            x->xvp = xcoord;
            x->yvp = ycoord;

            // render the stellar object point
            color = (i == tracking                 ? RED    :
                     i == ident                    ? ORANGE :
                     x->type == OBJTYPE_STELLAR    ? WHITE  :
                     x->type == OBJTYPE_SOLAR      ? YELLOW :
                     x->type == OBJTYPE_PLACE_MARK ? PURPLE :
                                                     BLACK);
            if (color == BLACK) {
                FATAL("BUG: obj %d '%s' invalid type %d\n", i, x->name, x->type);
            }
            sdl_render_point(pane, xcoord, ycoord, color, ptsz);
        }

        // draw cross hair
        sdl_render_line(pane,
                        pane->w/2-20, pane->h/2, pane->w/2+20, pane->h/2,
                        RED);
        sdl_render_line(pane,
                        pane->w/2, pane->h/2-20, pane->w/2, pane->h/2+20,
                        RED);

        // display names of the objects that have names
        fontsz = 18;  // objects
        for (i = 0; i < max_obj; i++) {
            obj_t * x = &obj[i];
            if (x->name[0] != '\0' && 
                x->xvp != NO_VALUE &&
                (x->type == OBJTYPE_SOLAR || sky_view_scale_tbl[sky_view_scale_tbl_idx] <= 10))
            {
                sdl_render_printf(pane,
                                x->xvp+6, x->yvp-ROW2Y(1,fontsz)/2,
                                fontsz, WHITE, BLACK, "%s", x->name);
            }
        }

        // print the angle diameter of the sky_view pane
        fontsz = 20;  // angle diamter
        sdl_render_printf(pane, 0, 0, fontsz, WHITE, BLACK,
                          "RADIUS %0.2g DEG", sky_view_scale_tbl[sky_view_scale_tbl_idx]);

        // register control events 
        rect_t loc = {0,0,pane->w,pane->h};
        sdl_register_event(pane, &loc, SDL_EVENT_MOUSE_MOTION, SDL_EVENT_TYPE_MOUSE_MOTION, pane_cx);
        sdl_register_event(pane, &loc, SDL_EVENT_MOUSE_WHEEL, SDL_EVENT_TYPE_MOUSE_WHEEL, pane_cx);

        return PANE_HANDLER_RET_NO_ACTION;
    }

    // -----------------------
    // -------- EVENT --------
    // -----------------------

    if (request == PANE_HANDLER_REQ_EVENT) {
        switch (event->event_id) {
        case SDL_EVENT_MOUSE_MOTION:
        case SDL_EVENT_KEY_UP_ARROW:
        case SDL_EVENT_KEY_DOWN_ARROW:
        case SDL_EVENT_KEY_LEFT_ARROW:
        case SDL_EVENT_KEY_RIGHT_ARROW: {
            int dx=0, dy=0;
            double max = sky_view_scale_tbl[sky_view_scale_tbl_idx];

            if (event->event_id == SDL_EVENT_MOUSE_MOTION) {
                dx = event->mouse_motion.delta_x;
                dy = event->mouse_motion.delta_y;
            } else if (event->event_id == SDL_EVENT_KEY_LEFT_ARROW) {
                dx = -5;
            } else if (event->event_id == SDL_EVENT_KEY_RIGHT_ARROW) {
                dx = +5;
            } else if (event->event_id == SDL_EVENT_KEY_UP_ARROW) {
                dy = -5;
            } else if (event->event_id == SDL_EVENT_KEY_DOWN_ARROW) {
                dy = +5;
            }

            if (dx == 0 && dy == 0) {
                break;
            }

            az_ctr -= dx * max / 250 / cos(el_ctr*DEG2RAD);
            if (az_ctr >  180) az_ctr -= 360;
            if (az_ctr < -180) az_ctr += 360;

            el_ctr += dy * max / 250;
            if (el_ctr > 90) el_ctr = 90;
            if (el_ctr < -90) el_ctr = -90;

            if (tracking != TRACKING_OFF) {
                tracking = TRACKING_RADEC;
                azel2radec(&tracking_ra, &tracking_dec, az_ctr, el_ctr, lst, latitude);
            }

            DEBUG("MOUSE MOTION dx=%d dy=%d  az_ctr=%f el_ctr=%f\n", dx, dy, az_ctr, el_ctr);
            return PANE_HANDLER_RET_DISPLAY_REDRAW; }
        case SDL_EVENT_MOUSE_WHEEL: {
            if (event->mouse_motion.delta_y > 0) {
                sky_view_scale_tbl_idx++;
            } else if (event->mouse_motion.delta_y < 0) {
                sky_view_scale_tbl_idx--;
            }

            if (sky_view_scale_tbl_idx < 0) sky_view_scale_tbl_idx = 0;
            if (sky_view_scale_tbl_idx >= MAX_SCALE_TBL) sky_view_scale_tbl_idx = MAX_SCALE_TBL-1;
            DEBUG("MOUSE WHEEL sky_view_scale_tbl_idx = %d\n", sky_view_scale_tbl_idx);
            return PANE_HANDLER_RET_DISPLAY_REDRAW; }
        }

        return PANE_HANDLER_RET_NO_ACTION;
    }

    // ---------------------------
    // -------- TERMINATE --------
    // ---------------------------

    if (request == PANE_HANDLER_REQ_TERMINATE) {
        free(vars);
        return PANE_HANDLER_RET_NO_ACTION;
    }

    // not reached
    assert(0);
    return PANE_HANDLER_RET_NO_ACTION;
}

// This routine converts the position of an object (az/el) from the sky_pane
// center az_ctr/el_ctr to xret/yret which are intended to have minimal distortion;.
// The xret/yret range from -1 to +1, where a value of -1 or +1 is equivalent to the max arg.
//
// For example: 
//   inputs: az_ctr=90  el_ctr=0
//           az=95  el=0  max=5
//   outputs: xret=1  yret=0
//
// This routine returns 
//   0 on success, or 
//  -1 if the az/el deviation from az_ctr/el_ctr by more than max
static int azel2xy(double az, double el, double max, double *xret, double *yret)
{
    double x,y,z;
    double incl, az_adj;
    double xrotate, yrotate, zrotate;
    double rot = el_ctr * DEG2RAD;

    // adjust az to be relative to az_ctr, and
    // convert elevation to inclination, 
    // both also converted to radians
    az_adj = (az - az_ctr) * DEG2RAD;
    incl   = (90 - el) * DEG2RAD;

    // convert max to radians
    max = max * DEG2RAD;

    // determine cartesion coords on a r=1 sphere, where
    // x is towards az_ctr
    // y is left/right, relative to az_ctr
    // z is up/down, relative to 0 elevation
    // reference: https://en.wikipedia.org/wiki/Spherical_coordinate_system
    x = sin(incl) * cos(az_adj);
    y = sin(incl) * sin(az_adj);
    z = cos(incl);

    // rotate about y by angle el_ctr
    // reference: https://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
    xrotate = x * cos(rot) + z * sin(rot);
    yrotate = y;
    zrotate = -x * sin(rot) + z * cos(rot);

    // return the x,y coords of az,el relative to az_ctr,el_ctr;
    // scaled to range -1..+1
    *xret = yrotate / sin(max);
    *yret = zrotate / sin(max);

    // debug prints
    DEBUG("az,el %f %f az_ctr,el_ctr %f %f  =>  %f %f (xrotate=%f)\n",
          az, el, az_ctr, el_ctr, *xret, *yret, xrotate);

    // return success when az/el deviation from az_ctr/el_ctr is less than max
    return xrotate >= cos(max) ? 0 : -1;
}

// -----------------  SKY TIME UTILS  -------------------------------------

static time_t __sky_time                    = 0;
static long   __sky_time_new_mode_req       = SKY_TIME_MODE_NONE;
static long   __sky_time_mode               = SKY_TIME_MODE_CURRENT;
static int    __sky_time_step_mode          = SKY_TIME_STEP_MODE_DELTA_T;
static double __sky_time_step_mode_hr_param = DEFAULT_SKY_TIME_STEP_MODE_DELTA_T;

static void sky_time_set_mode(int mode)
{
    __sky_time_new_mode_req = mode;
}

static void sky_time_get_mode(int * mode)
{
    *mode = __sky_time_mode;
}

static void sky_time_set_step_mode(int step_mode, double step_mode_hr_param)
{
    __sky_time_step_mode = step_mode;
    __sky_time_step_mode_hr_param = step_mode_hr_param;
}

static void sky_time_get_step_mode(int *step_mode, double *step_mode_hr_param)
{
    *step_mode = __sky_time_step_mode;
    *step_mode_hr_param = __sky_time_step_mode_hr_param;
}

static time_t sky_time_get_time(void) 
{
    time_t trise, tset;
    unsigned long time_now;

    static unsigned long time_of_last_call;
    static double jdss;  // jd for sunrise/sunset step mode

    // if this is an early call (less than 100 ms from last call) then
    // return the same sky_time as before; the reason being that when 
    // the display is being panned the display updates more frequently
    // than the normal 100 ms interval
    time_now = microsec_timer();
    if (time_now - time_of_last_call < 90000) {
        return __sky_time;
    }
    time_of_last_call = time_now;

    // handle request to change the time mode
    if (__sky_time_new_mode_req != SKY_TIME_MODE_NONE) {
        __sky_time_mode = __sky_time_new_mode_req;
        __sky_time_new_mode_req = SKY_TIME_MODE_NONE;
    }

    // determine return time based on time mode
    switch (__sky_time_mode) {
    case SKY_TIME_MODE_CURRENT:
        __sky_time = time(NULL);
        jdss = 0;
        break;
    case SKY_TIME_MODE_PAUSED:
        // no change to __sky_time.
        break;
    case SKY_TIME_MODE_FWD:
    case SKY_TIME_MODE_FWD_STEP:
        switch (__sky_time_step_mode) {
        case SKY_TIME_STEP_MODE_DELTA_T:
            __sky_time = __sky_time + __sky_time_step_mode_hr_param * 3600;
            jdss = 0;
            break;
        case SKY_TIME_STEP_MODE_SUNRISE:
            jdss = (jdss == 0 ? jdconv2(__sky_time) : jdss + 1);
            sunrise_sunset(jdss, &trise, &tset);
            __sky_time = trise + __sky_time_step_mode_hr_param * 3600;
            break;
        case SKY_TIME_STEP_MODE_SUNSET:
            jdss = (jdss == 0 ? jdconv2(__sky_time) : jdss + 1);
            sunrise_sunset(jdss, &trise, &tset);
            __sky_time = tset + __sky_time_step_mode_hr_param * 3600;
            break;
        case SKY_TIME_STEP_MODE_SIDDAY:
            __sky_time = __sky_time + SID_DAY_SECS;
            jdss = 0;
            break;
        case SKY_TIME_STEP_MODE_TIMEOFDAY:
            __sky_time = sky_time_tod_next(__sky_time, __sky_time_step_mode_hr_param);
            jdss = 0;
            break;
        default:
            FATAL("BUG: invalid __sky_time_step_mode %d\n", __sky_time_step_mode);
        }
        if (__sky_time_mode == SKY_TIME_MODE_FWD_STEP) {
            sky_time_set_mode(SKY_TIME_MODE_PAUSED);
        }
        break;
    case SKY_TIME_MODE_REV:
    case SKY_TIME_MODE_REV_STEP:
        switch (__sky_time_step_mode) {
        case SKY_TIME_STEP_MODE_DELTA_T:
            __sky_time = __sky_time - __sky_time_step_mode_hr_param * 3600;
            jdss = 0;
            break;
        case SKY_TIME_STEP_MODE_SUNRISE:
            jdss = (jdss == 0 ? jdconv2(__sky_time) : jdss - 1);
            sunrise_sunset(jdss, &trise, &tset);
            __sky_time = trise + __sky_time_step_mode_hr_param * 3600;
            break;
        case SKY_TIME_STEP_MODE_SUNSET:
            jdss = (jdss == 0 ? jdconv2(__sky_time) : jdss - 1);
            sunrise_sunset(jdss, &trise, &tset);
            __sky_time = tset + __sky_time_step_mode_hr_param * 3600;
            break;
        case SKY_TIME_STEP_MODE_SIDDAY:
            __sky_time = __sky_time - SID_DAY_SECS;
            jdss = 0;
            break;
        case SKY_TIME_STEP_MODE_TIMEOFDAY:
            __sky_time = sky_time_tod_prior(__sky_time, __sky_time_step_mode_hr_param);
            jdss = 0;
            break;
        default:
            FATAL("BUG: invalid __sky_time_step_mode %d\n", __sky_time_step_mode);
        }
        if (__sky_time_mode == SKY_TIME_MODE_REV_STEP) {
            sky_time_set_mode(SKY_TIME_MODE_PAUSED);
        }
        break;
    default:
        FATAL("BUG: invalid sky_time_mode %ld\n", __sky_time_mode);
    }

    // return the time
    return __sky_time;
} 

static time_t sky_time_tod_next(time_t t, double hr) 
{
    int year, month, day, hour, minute;
    double second;
    struct tm *tm, tm1;

    tm = gmtime(&t);
    year  = tm->tm_year + 1900;
    month = tm->tm_mon + 1;
    day   = tm->tm_mday;

    get_next_day(&year, &month, &day);

    hr2hms(hr, &hour, &minute, &second);

    memset(&tm1, 0, sizeof(struct tm));
    tm1.tm_year  = year-1900;   // based 1900
    tm1.tm_mon   = month-1;     // 0 to 11
    tm1.tm_mday  = day;
    tm1.tm_hour  = hour;
    tm1.tm_min   = minute;
    tm1.tm_sec   = second;
    tm1.tm_isdst = -1;

    return mktime(&tm1);
}

static time_t sky_time_tod_prior(time_t t, double hr) 
{
    int year, month, day, hour, minute;
    double second;
    struct tm *tm, tm1;

    tm = gmtime(&t);
    year  = tm->tm_year + 1900;
    month = tm->tm_mon + 1;
    day   = tm->tm_mday;

    get_prior_day(&year, &month, &day);

    hr2hms(hr, &hour, &minute, &second);

    memset(&tm1, 0, sizeof(struct tm));
    tm1.tm_year  = year-1900;   // based 1900
    tm1.tm_mon   = month-1;     // 0 to 11
    tm1.tm_mday  = day;
    tm1.tm_hour  = hour;
    tm1.tm_min   = minute;
    tm1.tm_sec   = second;
    tm1.tm_isdst = -1;

    return mktime(&tm1);
}

// -----------------  GENERAL UTIL  ---------------------------------------

static void reset(bool all_sky) 
{
    az_ctr  = 0;
    az_span = 360;
    if (all_sky) {
        el_ctr  = 0;
        el_span = 180;
    } else {
        el_ctr  = 45;
        el_span = 90; 
    }
    mag = DEFAULT_MAG;
    tracking = TRACKING_OFF;
    ident = IDENT_OFF;
    sky_view_scale_tbl_idx = 0;
    sky_time_set_mode(SKY_TIME_MODE_CURRENT);
    sky_time_set_step_mode(SKY_TIME_STEP_MODE_DELTA_T, DEFAULT_SKY_TIME_STEP_MODE_DELTA_T);
}

static void get_next_day(int * year, int * month, int * day)
{
                                    // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
    static char days_in_month[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

    days_in_month[2] = ((*year % 4) != 0) ? 28 : 29;  // not correct for 2100

    if (*day < days_in_month[*month]) {
        *day = *day + 1;
    } else {
        *day = 1;
        if (*month < 12) {
            *month = *month + 1;
        } else {
            *month = 1;
            *year = *year + 1;
        }
    }
}

static void get_prior_day(int * year, int * month, int * day)
{
                                    // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
    static char days_in_month[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

    days_in_month[2] = ((*year % 4) != 0) ? 28 : 29;  // not correct for 2100

    if (*day > 1) {
        *day = *day - 1;
    } else {
        if (*month > 1) {
            *month = *month - 1;
        } else {
            *month = 12;
            *year = *year - 1;
        }
        *day = days_in_month[*month];
    }
}
