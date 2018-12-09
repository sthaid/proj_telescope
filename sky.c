// XXX TODO
// - J2000 corresponds to  January 1, 2000, 11:58:55.816 UTC according to
//   https://en.wikipedia.org/wiki/Epoch_(astronomy)#Julian_years_and_J2000

// XXX organize favorites
// XXX update pleides to hour angle
// XXX make a much nicer display using smaller point sizes and displayng more 

#include "common.h"

//
// defines
//

#define MAX_STELLAR_OBJECT      200000
#define MAX_SOLAR_SYS_OBJECT    20 
#define MAX_NAME                32
#define MAX_INFO_TBL            100000

#define RAD2DEG (180. / M_PI)
#define DEG2RAD (M_PI / 180.)
#define HR2RAD  (M_PI / 12.)

// https://www.latlong.net/
#define MY_LAT    42.422986
#define MY_LONG  -71.623798

//
// typedefs
//

typedef struct {
    char  name[MAX_NAME];
    double ra;
    double dec;
    double mag;
} stellar_object_t;

typedef struct {
    char name[MAX_NAME];
    int max_info;
    int idx_info;
    struct info_s {
        time_t t;
        double ra;
        double dec;
        double mag;
    } info[MAX_INFO_TBL];
} solar_sys_object_t;

//
// variables
//

stellar_object_t    stellar_object[MAX_STELLAR_OBJECT];
int                 max_stellar_object;

solar_sys_object_t  solar_sys_object[MAX_SOLAR_SYS_OBJECT];
int                 max_solar_sys_object;

char *month_tbl[12] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };

//
// prototypes
//

int read_stellar_data(char * filename);
int read_solar_sys_data(char * filename);
int get_solar_sys_object_info(solar_sys_object_t *x, time_t t, double *ra, double *dec, double *mag);
char * gmtime_str(time_t t, char *str);

double jdconv(int yr, int mn, int day, double hour);
double jdconv2(time_t t);
double ct2lst(double lng, double jd);
void radec2azel(double *az, double *el, double ra, double dec, double lst, double lat);

void test_algorithms(void);
bool is_close(double act, double exp, double allowed_deviation, double * deviation);

// -----------------  SKY INIT  -------------------------------------------

int sky_init(void) 
{
    int ret;
    char str[100];

    INFO("sizeof(stellar_object)   = %ld MB\n", sizeof(stellar_object)/MB);
    INFO("sizeof(solar_sys_object) = %ld MB\n", sizeof(solar_sys_object)/MB);
    INFO("UTC now                  = %s UTC\n", gmtime_str(time(NULL),str));

    INFO("sizeof(M_PI) = %ld\n", sizeof(M_PI));
    INFO("sizeof(1.0) = %ld\n", sizeof(1.0));
    INFO("sizeof(1.0l) = %ld\n", sizeof(1.0l));

    ret = read_stellar_data("sky_data/hygdata_v3.csv");
    if (ret < 0) {
        return ret;
    }

    ret = read_stellar_data("sky_data/my_stellar_data.csv");
    if (ret < 0) {
        return ret;
    }

    ret = read_solar_sys_data("sky_data/solar_sys_data.csv");
    if (ret < 0) {
        return ret;
    }

    INFO("max_stellar_object   = %d\n", max_stellar_object);
    INFO("max_solar_sys_object = %d\n", max_solar_sys_object);

    test_algorithms();

    return 0;
}

int read_stellar_data(char * filename)
{
    // csv file format:
    //   id,hip,hd,hr,gl,bf,proper,ra,dec,dist,pmra,pmdec,rv,mag,
    //      absmag,spect,ci,x,y,z,vx,vy,vz,rarad,decrad,pmrarad,pmdecrad,bayer,
    //      flam,con,comp,comp_primary,base,lum,var,var_min,var_max

    #define GET_FIELD(field) \
        do { \
            field = s; \
            s = strchr(s,','); \
            if (s == NULL) { \
                ERROR("filename=%s line=%d field=%s\n", filename, line, #field); \
                return -1; \
            } \
            *s = '\0'; \
            s++; \
        } while (0)

    FILE *fp;
    int line=1, num_added=0;
    char str[10000], *s;
    char *id_str     __attribute__ ((unused)); 
    char *hip_str    __attribute__ ((unused));
    char *hd_str     __attribute__ ((unused));
    char *hr_str     __attribute__ ((unused));
    char *gl_str     __attribute__ ((unused));
    char *bf_str     __attribute__ ((unused));
    char *dist_str   __attribute__ ((unused));
    char *pmra_str   __attribute__ ((unused));
    char *pmdec_str  __attribute__ ((unused));
    char *rv_str     __attribute__ ((unused));
    char *proper_str, *ra_str, *dec_str, *mag_str;
    double ra, dec, mag;

    // open file
    fp = fopen(filename, "r");
    if (fp == NULL) {
        ERROR("failed to open %s\n", filename);
        return -1;
    }

    // read and parse all lines
    while (fgets(str, sizeof(str), fp) != NULL) {
        if (max_stellar_object >= MAX_STELLAR_OBJECT) {
            ERROR("filename=%s line=%d stellar_object table is full\n", filename, line);
            return -1;
        }

        s=str;
        GET_FIELD(id_str); 
        GET_FIELD(hip_str);
        GET_FIELD(hd_str);
        GET_FIELD(hr_str);
        GET_FIELD(gl_str);
        GET_FIELD(bf_str);
        GET_FIELD(proper_str);
        GET_FIELD(ra_str);
        GET_FIELD(dec_str);
        GET_FIELD(dist_str);
        GET_FIELD(pmra_str);
        GET_FIELD(pmdec_str);
        GET_FIELD(rv_str);
        GET_FIELD(mag_str);

        if (line == 1) {
            if (strcmp(proper_str, "proper") || strcmp(ra_str, "ra") || strcmp(dec_str, "dec") || strcmp(mag_str, "mag")) {
                ERROR("csv file header line incorrect, proper='%s' ra=%s dec=%s mag=%s\n", proper_str, ra_str, dec_str, mag_str);
                return -1;
            }
            line++;
            continue;
        } 

        if (sscanf(ra_str, "%lf", &ra) != 1) {
            ERROR("filename=%s line=%d invalid ra='%s'\n", filename, line, ra_str);
            return -1;
        }
        if (sscanf(dec_str, "%lf", &dec) != 1) {
            ERROR("filename=%s line=%d invalid dec='%s'\n", filename, line, dec_str);
            return -1;
        }
        if (sscanf(mag_str, "%lf", &mag) != 1) {
            ERROR("filename=%s line=%d invalid mag='%s'\n", filename, line, mag_str);
            return -1;
        }

        strncpy(stellar_object[max_stellar_object].name, proper_str, MAX_NAME);
        stellar_object[max_stellar_object].ra  = ra * 15.;
        stellar_object[max_stellar_object].dec = dec;
        stellar_object[max_stellar_object].mag = mag;

        INFO("%16s %10.4f %10.4f %10.4f\n", proper_str, ra, dec, mag);
        max_stellar_object++;

        num_added++;
        line++;
    }

    // close file
    fclose(fp);

    // success
    INFO("added %d stellar_objects from %s\n", num_added, filename);
    return 0;
}

int read_solar_sys_data(char * filename)
{
    // format, example:
    //   # Venus
    //    2018-Dec-01 00:00, , ,207.30643, -9.79665,  -4.87,  1.44,

    FILE *fp;
    int line=1, num_added=0, i, len;
    solar_sys_object_t *x = NULL;
    char str[10000], *s;
    char *date_str, *ra_str, *dec_str, *mag_str;
    char *not_used_str __attribute__ ((unused));
    double ra, dec, mag, lst;
    time_t t;

    // open
    fp = fopen(filename, "r");
    if (fp == NULL) {
        ERROR("failed to open %s\n", filename);
        return -1;
    }

    // read and parse all lines
    while (fgets(str, sizeof(str), fp) != NULL) {
        s=str;

        // first line in file must be an object name
        if (line == 1 && s[0] != '#') {
            ERROR("first line must contain solar_sys_object.name, '%s'\n", str);
            return -1;
        }

        // if this line is object name then start a new object
        if (s[0] == '#') {
            // check for too many
            if (max_solar_sys_object >= MAX_SOLAR_SYS_OBJECT) {
                ERROR("filename=%s line=%d solar_sys_object table is full\n", filename, line);
                return -1;
            }

            // get ptr to the solar_sys_object being initialized
            x = &solar_sys_object[max_solar_sys_object];

            // save the name of the solar_sys_object, and remove trailing newline
            strncpy(x->name, s+2, MAX_NAME);
            len = strlen(x->name);
            if (len > 0 && x->name[len-1] == '\n') x->name[len-1] = 0;

            // update counters
            max_solar_sys_object++;
            num_added++;
            line++;
            continue;
        } 

        // get fields for the object currently being input
        GET_FIELD(date_str);
        GET_FIELD(not_used_str);
        GET_FIELD(not_used_str);
        GET_FIELD(ra_str);
        GET_FIELD(dec_str);
        GET_FIELD(mag_str);

        // sometimes magnitude is not-avail, such as when Mercury is on the
        // other side of the sun; in this case set magnitude to 990
        if (strstr(mag_str, "n.a.")) {
            mag_str = "99";
        }

        // convert utc date_str to t
        // - example format: 2018-Dec-01 00:00,
        {
        int year, month, day, hour, minute, cnt;
        static char month_str[10];
        struct tm tm;
        cnt = sscanf(date_str, "%d-%c%c%c-%d %d:%d", &year, month_str+0, month_str+1, month_str+2, &day, &hour, &minute);
        if (cnt != 7) {
            ERROR("filename=%s line=%d invalid date_str='%s'\n", filename, line, date_str);
            return -1;
        }
        for (month = 0; month < 12; month++) {
            if (strcmp(month_str, month_tbl[month]) == 0) {
                break;
            }
        }
        if (month == 12) {
            ERROR("filename=%s line=%d invalid month_str='%s'\n", filename, line, month_str);
            return -1;
        }
        memset(&tm,0,sizeof(tm));
        tm.tm_min   = minute;
        tm.tm_hour  = hour;
        tm.tm_mday  = day;
        tm.tm_mon   = month;       // 0 to 11
        tm.tm_year  = year-1900;   // based 1900
        t = timegm(&tm);
        }

        // convert the righ-ascension, declination, and magniture strings to doubleing point
        if (sscanf(ra_str, "%lf", &ra) != 1) {
            ERROR("filename=%s line=%d invalid ra='%s'\n", filename, line, ra_str);
            return -1;
        }
        if (sscanf(dec_str, "%lf", &dec) != 1) {
            ERROR("filename=%s line=%d invalid dec='%s'\n", filename, line, dec_str);
            return -1;
        }
        if (sscanf(mag_str, "%lf", &mag) != 1) {
            ERROR("filename=%s line=%d invalid mag='%s'\n", filename, line, mag_str);
            return -1;
        }
        
        // fill in the info table for the object currently being input
        if (x->max_info >= MAX_INFO_TBL) {
            ERROR("filename=%s line=%d solar_sys_object '%s' info table is full\n", filename, line, x->name);
            return -1;
        }
        x->info[x->max_info].t   = t;
        x->info[x->max_info].ra  = ra;
        x->info[x->max_info].dec = dec;
        x->info[x->max_info].mag = mag;
        x->max_info++;

        // update line counter, which is used to identify which line in filename is in error
        line++;
    }

    // close
    fclose(fp);

    // print how many solar sys objects added
    INFO("added %d solar_sys_objects from %s\n", num_added, filename);

    // print the list of solar_sys objects that have been input
    //      xxxxxxxxxxxxxxxx xxxxxxxxxx xxxxxxxxxx xxxxxxxxxx xxxxxxxxxx xxxxxxxxxx
    t = time(NULL);
    lst = ct2lst(MY_LONG, jdconv2(t));
    printf("            NAME         RA        DEC        MAG         AZ         EL\n");
    for (i = 0; i < max_solar_sys_object; i++) {
        solar_sys_object_t * x = &solar_sys_object[i];
        double ra, dec, mag, az, el;

        get_solar_sys_object_info(x, t, &ra, &dec, &mag);
        radec2azel(&az, &el, ra, dec, lst, MY_LAT);

        printf("%16s %10.4f %10.4f %10.1f %10.4f %10.4f\n", x->name, ra, dec, mag, az, el);
    }

    // success
    return 0;
}

// -----------------  PANE HANDLERS  --------------------------------------
// AAAAAAAAAAAAA
// XXX 
// - clean up
// - hover mouse will OR click will iluminate obj and display info in ctrl pane
// - ctrl pane
//    . info display
//    . mag select
//    . reset (instead of home)
// XXX  DONE
// - make room for ctrl pane
// - panning when zoomed in
// - display az and el on borders
// - display other grid lines

double az_ctr  = 0;
double az_span = 360;
double el_ctr  = 0;
double el_span = 180;
double mag = 8;

int sky_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event) 
{
    struct {
        int not_used;
    } * vars = pane_cx->vars;
    rect_t * pane = &pane_cx->pane;

    #define SDL_EVENT_MOUSE_MOTION   (SDL_EVENT_USER_DEFINED + 0)
    #define SDL_EVENT_MOUSE_WHEEL    (SDL_EVENT_USER_DEFINED + 1)

    // ----------------------------
    // -------- INITIALIZE --------
    // ----------------------------

    if (request == PANE_HANDLER_REQ_INITIALIZE) {
        vars = pane_cx->vars = calloc(1,sizeof(*vars));
        INFO("PANE x,y,w,h  %d %d %d %d\n",
            pane->x, pane->y, pane->w, pane->h);
        return PANE_HANDLER_RET_NO_ACTION;
    }

    // ------------------------
    // -------- RENDER --------
    // ------------------------

    if (request == PANE_HANDLER_REQ_RENDER) {
        rect_t * pane = &pane_cx->pane;

        double min_az = az_ctr - az_span / 2.;
        double max_az = az_ctr + az_span / 2.;
        double min_el = el_ctr - el_span / 2.;
        double max_el = el_ctr + el_span / 2.;
        double k_el = (pane->h) / (max_el - min_el);
        double k_az = (pane->w) / (min_az - max_az);

        const int fontsz=30;

        int x, y, i, ptsz;
        double az, el, lst;
        time_t t;
        double grid_sep, first_az_line, first_el_line;

        // get local sidereal time        
        t = time(NULL);
        lst = ct2lst(MY_LONG, jdconv2(t));

        // draw points for stellar objects
        for (i = 0; i < max_stellar_object; i++) {
            stellar_object_t * obj = &stellar_object[i];

            // magnitude too dim then skip
            if (obj->mag > mag) {
                continue;
            }

            // get az/el from ra/dec,
            // sanity check az and el, and
            // convert az to range -180 to + 180
            radec2azel(&az, &el, obj->ra, obj->dec, lst, MY_LAT);
            if (el < -90 || el > 90 || az < 0 || az > 360) {
                WARN("stellar obj %d '%s' at ra=%f dec=%f has invalid az=%f or el=%f\n",
                     i, obj->name, obj->ra, obj->dec, az, el);
                continue;
            }
            if (az > 180)  az -= 360;

            // adjust az, up or down by 360, to try to get it in range min_az to maz_az;
            // if unable to get az in range min_az to maz_az then skip
            if (az > max_az) {
                while (az > max_az) az -= 360;
            } else if (az < min_az) {
                while (az < min_az) az += 360;
            }
            if (az < min_az || az > max_az) {
                continue;
            }

            // determine display point size from object's apparent magnitude
            // XXX verify, and is 8 a good choice
            ptsz = 6 - obj->mag;  
            if (ptsz < 0) ptsz = 0;
            if (ptsz > 9) ptsz = 9;

            // convert az/el to pane coordinates; 
            // if coords are out of the pane then skip
            y = 0 + k_el * (max_el - el);
            x = 0 + k_az * (min_az - az);
            if (x < 0 || x >= pane->w || y < 0 || y >= pane->h) {
                continue;
            }

            // render the stellar object point
            sdl_render_point(pane, x, y, WHITE, ptsz);
        }

        // draw grid
        // XXX location of azimuth string
        grid_sep = el_span > 120 ? 45 :
                   el_span > 60  ? 20 :
                   el_span > 30  ? 10 :
                   el_span > 10  ? 5 :
                   el_span > 4   ? 2 :
                   el_span > 2   ? 1 :
                   el_span > 1   ? 0.5 :
                                   0.25;
        first_az_line = floor(min_az/grid_sep) * grid_sep;
        for (az = first_az_line; az < max_az; az += grid_sep) {
            x = 0 + k_az * (min_az - az);
            sdl_render_line(pane, x, 0, x, pane->h-1, BLUE);
            sdl_render_printf(pane, x-COL2X(2,fontsz), pane->h-ROW2Y(1,fontsz), fontsz, BLUE, BLACK, "%g", az);
        }
        first_el_line = floor(min_el/grid_sep) * grid_sep;
        for (el = first_el_line; el < max_el; el += grid_sep) {
            y = 0 + k_el * (max_el - el);
            sdl_render_line(pane, 0, y, pane->w-1, y, BLUE);
            sdl_render_printf(pane, 0, y-ROW2Y(1,fontsz)/2, fontsz, BLUE, BLACK, "%g ", el);
        }

#if 0
        // draw a purple point at sidereal north
        // XXX instead just add to my sky data
        y = 0 + k_el * (max_el - MY_LAT);
        x = 0 + k_az * (min_az - 0);
        if (x >= 0 && x < pane->w && y >= 0 && y < pane->h) {
            sdl_render_point(pane, x, y, PURPLE, 9);
        }
#endif

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
        case SDL_EVENT_MOUSE_MOTION: {
            int dx = event->mouse_motion.delta_x;
            int dy = event->mouse_motion.delta_y;
            az_ctr -= dx * (az_span / 1800.);
            if (az_ctr > 180) az_ctr = 180;
            if (az_ctr < -180) az_ctr = -180;
            el_ctr += dy * (el_span / 900.);
            if (el_ctr > 90) el_ctr = 90;
            if (el_ctr < -90) el_ctr = -90;
            //printf("%d %d  %f %f\n", dx, dy, az_ctr, el_ctr);
            return PANE_HANDLER_RET_DISPLAY_REDRAW; }
        case SDL_EVENT_MOUSE_WHEEL: {
            int dy = event->mouse_motion.delta_y;
            if (dy < 0) {
                az_span *= 1.1;
                el_span *= 1.1;
            }
            if (dy > 0 && az_span > 1) {
                az_span /= 1.1;
                el_span /= 1.1;
            }
            //XXX printf("%d  %f %f\n", dy, az_span, el_span);
            return PANE_HANDLER_RET_DISPLAY_REDRAW; }
        case SDL_EVENT_KEY_HOME: {
            az_ctr  = 0;
            az_span = 360;
            el_ctr  = 0;
            el_span = 180;
            return PANE_HANDLER_RET_DISPLAY_REDRAW; }
#if 0
        case SDL_EVENT_KEY_END:
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_PGUP:
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_PGDN:
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_UP_ARROW:
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_DOWN_ARROW:
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
#endif
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

int sky_ctl_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event) 
{
    struct {
        int not_used;
    } * vars = pane_cx->vars;
    rect_t * pane = &pane_cx->pane;

    #define SDL_EVENT_MOUSE_MOTION   (SDL_EVENT_USER_DEFINED + 0)
    #define SDL_EVENT_MOUSE_WHEEL    (SDL_EVENT_USER_DEFINED + 1)

    // ----------------------------
    // -------- INITIALIZE --------
    // ----------------------------

    if (request == PANE_HANDLER_REQ_INITIALIZE) {
        vars = pane_cx->vars = calloc(1,sizeof(*vars));
        INFO("PANE x,y,w,h  %d %d %d %d\n",
            pane->x, pane->y, pane->w, pane->h);
        return PANE_HANDLER_RET_NO_ACTION;
    }

    // ------------------------
    // -------- RENDER --------
    // ------------------------

    if (request == PANE_HANDLER_REQ_RENDER) {
        //rect_t * pane = &pane_cx->pane;
        return PANE_HANDLER_RET_NO_ACTION;
    }

    // -----------------------
    // -------- EVENT --------
    // -----------------------

    if (request == PANE_HANDLER_REQ_EVENT) {
#if 0
        switch (event->event_id) {
        case SDL_EVENT_MOUSE_MOTION:
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_MOUSE_WHEEL:
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_HOME:
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_END:
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_PGUP:
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_PGDN:
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_UP_ARROW:
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_DOWN_ARROW:
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        }
#endif

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

// -----------------  GENERAL UTIL  ---------------------------------------

int get_solar_sys_object_info(solar_sys_object_t *x, time_t t, double *ra, double *dec, double *mag)
{
    struct info_s *info     = x->info;
    int            max_info = x->max_info;
    int            idx      = x->idx_info;
    char           str[100];

    // interpolated_val = v0 + ((v1 - v0) / (t1 - t0)) * (t - t0)
    #define INTERPOLATE(t,v0,v1,t0,t1) \
        ((double)(v0) + (((double)(v1) - (double)(v0)) / ((double)(t1) - (double)(t0))) * ((double)(t) - (double)(t0)))

    if (t >= info[idx].t && t <= info[idx+1].t) {
        goto interpolate;
    }

    while (t < info[idx].t) {
        idx--;
        if (idx < 0) {
            ERROR("time %s too early for solar_sys_object %s\n", gmtime_str(t,str), x->name);
            return -1;
        }
    }

    while (t > info[idx+1].t) {
        idx++;
        if (idx > max_info-2) {
            ERROR("time %s too large for solar_sys_object %s\n", gmtime_str(t,str), x->name);
            return -1;
        }
    }

    if (t < info[idx].t || t > info[idx+1].t) {
        ERROR("bug t=%ld info[%d].t=%ld info[%d].t=%ld\n",
              t, idx, info[idx].t, idx+1, info[idx+1].t);
        return -1;
    }

interpolate:
    // determine ra and dec return values by interpolation;
    // don't bother interpolating mag
    *ra  = INTERPOLATE(t, info[idx].ra, info[idx+1].ra, info[idx].t, info[idx+1].t); 
    *dec = INTERPOLATE(t, info[idx].dec, info[idx+1].dec, info[idx].t, info[idx+1].t); 
    *mag = info[idx].mag;
        
    // save idx hint
    x->idx_info = idx;

    // return success
    return 0;
}

char *gmtime_str(time_t t, char *str)
{
    struct tm *tm;

    // format:  yyyy-mmm-dd hh:mm
    // example: 2018-Dec-01 00:00
    tm = gmtime(&t);
    sprintf(str, "%4d-%s-%2.2d %2.2d:%2.2d",
            tm->tm_year+1900, 
            month_tbl[tm->tm_mon],
            tm->tm_mday,
            tm->tm_hour,
            tm->tm_min);
    return str;
}

char * hr_str(double hr, char *str)
{
    int hour, min;
    double seconds = hr * 3600;

    hour = seconds / 3600;
    seconds -= 3600 * hour;
    min = seconds / 60;
    seconds -= min * 60;
    sprintf(str, "%2.2d:%2.2d:%5.2f", hour, min, seconds);
    return str;
}

// -----------------  CONVERT RA/DEC TO AZ/EL UTILS  ----------------------

// INFO
// - https://en.wikipedia.org/wiki/Epoch_(reference_date)#J2000.0
//   The current standard epoch is called "J2000.0" This is defined by 
//   international agreement to be equivalent to:
//   . The Gregorian date January 1, 2000 at approximately 12:00 GMT (Greenwich Mean Time).
//  . The Julian date 2451545.0 TT (Terrestrial Time).[8]
//  . January 1, 2000, 11:59:27.816 TAI (International Atomic Time).[9]
//  . January 1, 2000, 11:58:55.816 UTC (Coordinated Universal Time).[a]
//
// - websites containing conversion algorithms
//   https://paulplusx.wordpress.com/2016/03/02/rtpts_azalt/
//   https://www.mathworks.com/matlabcentral/fileexchange/26458-convert-right-ascension-and-declination-to-azimuth-and-elevation
//   http://cosmology.berkeley.edu/group/cmbanalysis/forecast/idl/radec2azel.pro
//   https://idlastro.gsfc.nasa.gov/ftp/pro/astro/ct2lst.pro
//   https://idlastro.gsfc.nasa.gov/ftp/pro/astro/jdcnv.pro
//
// - julian date converter
//   https://www.aavso.org/jd-calculator
//
// - sidereal time converter
//   https://tycho.usno.navy.mil/sidereal.html
    

//https://idlastro.gsfc.nasa.gov/ftp/pro/astro/jdcnv.pro
//Converts Gregorian dates to Julian days
double jdconv(int yr, int mn, int day, double hour)
{
    int L, julian;

    L = (mn-14)/12;    // In leap years, -1 for Jan, Feb, else 0
    julian = day - 32075 + 1461*(yr+4800+L)/4 + 
             367*(mn - 2-L*12)/12 - 3*((yr+4900+L)/100)/4;

    return (double)julian + (hour/24.) - 0.5;
}

double jdconv2(time_t t)
{
    struct tm * tm    = gmtime(&t);
    int         year  = tm->tm_year + 1900;
    int         month = tm->tm_mon + 1;
    int         day   = tm->tm_mday;
    double      hour  = (double)tm->tm_hour + (double)tm->tm_min / 60 + (double)tm->tm_sec / 3600;

    return jdconv(year, month, day, hour);
}

// https://idlastro.gsfc.nasa.gov/ftp/pro/astro/ct2lst.pro
// To convert from Local Civil Time to Local Mean Sidereal Time.
double ct2lst(double lng, double jd)
{
    static double c[4] = {280.46061837, 360.98564736629, 0.000387933, 38710000.0} ;
    static double jd2000 = 2451545.0;

    double t0, t, theta, lst;

    t0 = jd - jd2000;
    t = t0 / 36525;

    // Compute GST in seconds.
    theta = c[0] + (c[1] * t0) + t * t * (c[2] - t / c[3]);

    // Compute LST in hours.
    lst = (theta + lng) / 15.0;

    if (lst < 0) {
        lst = -lst;
        lst = lst - 24 * ((int)lst / 24);
        lst = 24. - lst;
    } else {
        lst = lst - 24 * ((int)lst / 24);
    }

    return lst;
}

//  http://cosmology.berkeley.edu/group/cmbanalysis/forecast/idl/radec2azel.pro
//  To convert from celestial coordinates (right ascension-declination)
//  to horizon coordinates (azimuth-elevation)
void radec2azel(double *az, double *el, double ra, double dec, double lst, double lat)
{
    double rha, rdec, rel, raz, rlat;

    // get declination, latitude, and hourangle in radians
    rdec = dec * DEG2RAD;
    rlat = lat * DEG2RAD;
    rha = (lst * HR2RAD) - (ra * DEG2RAD);

    // working out elevation
    rel = asin( sin(rdec)*sin(rlat)+cos(rdec)*cos(rha)*cos(rlat) );

    // working out azimuth
    raz = atan2( cos(rdec)*sin(rha),
                 -sin(rdec)*cos(rlat)+cos(rdec)*cos(rha)*sin(rlat)
                     );

    // return az/el in degrees
    // note: I added the '+ 180'
    *az = raz * RAD2DEG + 180.;
    *el = rel * RAD2DEG;
}

// -----------------  TEST ALGORITHMS  ------------------------------------

void test_algorithms(void) 
{
    double jd, lst, ra, dec, az, el, lat, lng;
    double deviation;
    int i;
    unsigned long start_us, duration_us;
    time_t t;

    // test jdconv ...
    // https://www.aavso.org/jd-calculator; also
    // refer to https://en.wikipedia.org/wiki/Epoch_(astronomy)#Julian_years_and_J2000
    jd =  jdconv(2000,1,1,12);
    if (jd != 2451545.0) {
        ERROR("jd %f should be 2451545.0\n", jd);
    }

    // test local sidereal time ...
    // from date cmd: Fri Dec  7 20:53:37 EST 2018
    // from https://tycho.usno.navy.mil/cgi-bin/sidereal-post.sh
    //    02:12:44.9 LST         2+12/60+44.9/3600 = 2.21247
    //    Longitude -72 00 00
    jd = jdconv(2018,12,8,1.8936);  
    lst = ct2lst(-72, jd);
    if (!is_close(lst, 2.21247, 0.000010, &deviation)) {
        ERROR("lst deviation = %f, exceeds limit\n", deviation);
    }
    INFO("lst deviation = %f\n", deviation);

    // this webiste has a radec2azel convert code, which I did not use; 
    // however I did use the sample problem provided in the comments ...
    // https://www.mathworks.com/matlabcentral/fileexchange/26458-convert-right-ascension-and-declination-to-azimuth-and-elevation
    // 
    // Worked Example: http://www.stargazing.net/kepler/altaz.html
    // [Az El] = RaDec2AzEl(344.95,42.71667,52.5,-1.91667,'1997/03/14 19:00:00')
    // [311.92258 22.40100] = RaDec2AzEl(344.95,42.71667,52.5,-1.91667,'1997/03/14 19:00:00')
    ra  = 344.95;
    dec = 42.71667;
    lat = 52.5;
    lng = -1.91667;
    jd = jdconv(1997, 3, 14, 19);
    lst = ct2lst(lng, jd);
    radec2azel(&az, &el, ra, dec, lst, lat);
    DEBUG("az = %f   el = %f\n", az, el);
    if (!is_close(az, 311.92258, 0.000010, &deviation)) {
        INFO("az deviation = %f, exceeds limit\n", deviation);
    }
    INFO("az deviation = %f\n", deviation);
    if (!is_close(el, 22.40100, 0.000010, &deviation)) {
        INFO("el deviation = %f, exceeds limit\n", deviation);
    }
    INFO("el deviation = %f\n", deviation);

    // time how long to convert all stellar objects to az/el
    start_us = microsec_timer();
    t = time(NULL);
    lst = ct2lst(MY_LONG, jdconv2(t));
    for (i = 0; i < max_stellar_object; i++) {
        stellar_object_t * x = &stellar_object[i];
        radec2azel(&az, &el, x->ra, x->dec, lst, MY_LAT);
    }
    duration_us = microsec_timer() - start_us;
    INFO("radec2azel perf: %d objects in %ld ms\n", max_stellar_object, duration_us/1000);

    INFO("tests passed\n");
}

bool is_close(double act, double exp, double allowed_deviation, double * deviation)
{
    bool is_close;
    *deviation = fabs((act - exp) / exp);
    is_close = (*deviation < 0.000010);
    return is_close;
}
