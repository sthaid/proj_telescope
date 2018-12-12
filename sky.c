// XXX use arrow keys for scolling too, and use other keys for zoom

// XXX do a better job of displaying grid coord numbers
//      - they could be smaller and better positioned

// XXX TODO
// - J2000 corresponds to  January 1, 2000, 11:58:55.816 UTC according to
//   https://en.wikipedia.org/wiki/Epoch_(astronomy)#Julian_years_and_J2000

// XXX organize favorites

// XXX what is Sol in hygdata, should eliminate

// XXX why sometimes sdl key events not working?

// XXX add code to sanity check that names dont begin or end with a space

// XXX add code to support PLACE_MARK

// XXX add code to support temporary PLACE_MARK,  when right clicked somewhere without something close by ??????

// XXX grid sep needs work when elspan is 90

// XXX ctl pane layout
// - ctl pane perhaps goes below sky

//XXX place marks should be displayed independent of mag

#include "common.h"

//
// defines
//

#define MAX_OBJ              200000
#define MAX_OBJ_NAME         32
#define MAX_SS_OBJ_INFO_TBL  100000  // XXX make this dynamic

#define RAD2DEG (180. / M_PI)
#define DEG2RAD (M_PI / 180.)
#define HR2RAD  (M_PI / 12.)

#define MY_LAT    42.422986   // from https://www.latlong.net/
#define MY_LONG  -71.623798   // for Bolton Mass USA

#define NO_VALUE 9999

#define OBJTYPE_NONE             0
#define OBJTYPE_STELLAR          1
#define OBJTYPE_SOLAR            2
#define OBJTYPE_PLACE_MARK       3

#define DEFAULT_MAG 7.

//
// typedefs
//

typedef struct {
    int max_info;
    int idx_info;
    struct info_s {
        time_t t;
        double ra;
        double dec;
        double mag;
    } info[MAX_SS_OBJ_INFO_TBL];
} solar_sys_obj_info_t;

typedef struct {
    char name[MAX_OBJ_NAME];
    int type;
    double ra;
    double dec;
    double mag;
    double az;
    double el;
    int x;
    int y;
    solar_sys_obj_info_t * ssinfo;
} obj_t;

//
// variables
//

obj_t obj[MAX_OBJ];
int   max_obj;

char *month_tbl[12] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };

//
// prototypes
//

int read_stellar_data(char * filename);
int read_solar_sys_data(char * filename);
int read_place_marks(char * filename);
int compute_ss_obj_ra_dec_mag(obj_t *x, time_t t);
char * gmtime_str(time_t t, char *str);

double jdconv(int yr, int mn, int day, double hour);
double jdconv2(time_t t);
double ct2lst(double lng, double jd);
int radec2azel(double *az, double *el, double ra, double dec, double lst, double lat);

void test_algorithms(void);
bool is_close(double act, double exp, double allowed_deviation, double * deviation);

// -----------------  SKY INIT  -------------------------------------------

int sky_init(void) 
{
    int ret;
    char str[100];

    INFO("sizeof(obj)  = %ld MB\n", sizeof(obj)/MB);
    INFO("sizeof(M_PI) = %ld\n", sizeof(M_PI));
    INFO("sizeof(1.0)  = %ld\n", sizeof(1.0));
    INFO("sizeof(1.0l) = %ld\n", sizeof(1.0l));
    INFO("UTC now      = %s UTC\n", gmtime_str(time(NULL),str));

    ret = read_stellar_data("sky_data/hygdata_v3.csv");
    if (ret < 0) {
        return ret;
    }

    ret = read_solar_sys_data("sky_data/solar_sys_data.csv");
    if (ret < 0) {
        return ret;
    }

    ret = read_place_marks("sky_data/place_marks.dat");
    if (ret < 0) {
        return ret;
    }

    INFO("max_object  = %d\n", max_obj);

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
// XXX ^^^ needs to remove leading and trailing spaces

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
        if (max_obj >= MAX_OBJ) {
            ERROR("filename=%s line=%d obj table is full\n", filename, line);
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

        if (strcmp(proper_str, "Sol") == 0) {
            INFO("XXX skip Sol\n");
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

        strncpy(obj[max_obj].name, proper_str, MAX_OBJ_NAME);
        obj[max_obj].type = OBJTYPE_STELLAR;
        obj[max_obj].ra       = ra * 15.;
        obj[max_obj].dec      = dec;
        obj[max_obj].mag      = mag;
        obj[max_obj].az       = NO_VALUE;
        obj[max_obj].el       = NO_VALUE;
        obj[max_obj].x        = NO_VALUE;
        obj[max_obj].y        = NO_VALUE;
        obj[max_obj].ssinfo   = NULL;

        //INFO("%16s %10.4f %10.4f %10.4f\n", proper_str, ra, dec, mag);
        max_obj++;

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
    int line=1, num_added=0, i, len, ret;
    obj_t *x = NULL;
    solar_sys_obj_info_t *ssinfo = NULL;
    char str[10000], *s, *name;
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
            ERROR("first line must contain solar sys object name, '%s'\n", str);
            return -1;
        }

        // if this line is object name then start a new object
        if (s[0] == '#') {
            // check for too many
            if (max_obj >= MAX_OBJ) {
                ERROR("filename=%s line=%d obj table is full\n", filename, line);
                return -1;
            }

            // get ptr to the name of the solar_sys_object, and remove trailing newline
            name = s + 2;
            len = strlen(name);
            if (len > 0 && name[len-1] == '\n') name[len-1] = 0;

            // init obj fields
            x = &obj[max_obj];
            ssinfo = calloc(1,sizeof(solar_sys_obj_info_t));
            strncpy(x->name, name, MAX_OBJ_NAME);
            x->type = OBJTYPE_SOLAR;
            x->ra       = NO_VALUE;
            x->dec      = NO_VALUE;
            x->mag      = NO_VALUE;
            x->az       = NO_VALUE;
            x->el       = NO_VALUE;
            x->x        = NO_VALUE;
            x->y        = NO_VALUE;
            x->ssinfo   = ssinfo;

            // update counters
            max_obj++;
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
        // other side of the sun; in this case set magnitude to 99
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
        if (ssinfo->max_info >= MAX_SS_OBJ_INFO_TBL) {
            ERROR("filename=%s line=%d solar sys obj '%s' info table is full\n", filename, line, x->name);
            return -1;
        }
        ssinfo->info[ssinfo->max_info].t   = t;
        ssinfo->info[ssinfo->max_info].ra  = ra;
        ssinfo->info[ssinfo->max_info].dec = dec;
        ssinfo->info[ssinfo->max_info].mag = mag;
        ssinfo->max_info++;

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
    INFO("            NAME         RA        DEC        MAG         AZ         EL\n");
    for (i = 0; i < max_obj; i++) {
        obj_t *x = &obj[i];
        double az, el;

        if (x->type != OBJTYPE_SOLAR) {
            continue;
        }

        compute_ss_obj_ra_dec_mag(x, t);
        ret = radec2azel(&az, &el, x->ra, x->dec, lst, MY_LAT);
        if (ret != 0) {
            ERROR("radec2azel failed\n");
            return -1;
        }

        INFO("%16s %10.4f %10.4f %10.1f %10.4f %10.4f\n", x->name, x->ra, x->dec, x->mag, az, el);
    }

    // success
    return 0;
}

int read_place_marks(char *filename)
{
    FILE * fp;
    int line=1, num_added=0, cnt;
    char str[1000], name[100];
    double ra, dec;

    // open
    fp = fopen(filename, "r");
    if (fp == NULL) {
        ERROR("failed to open %s\n", filename);
        return -1;
    }

    // read and parse all lines
    while (fgets(str, sizeof(str), fp) != NULL) {
        if (str[0] == '#') {
            continue;
        }

        cnt = sscanf(str, "%s %lf %lf\n", name, &ra, &dec);
        if (cnt != 3) {
            ERROR("filename=%s line=%d invalid, scan cnt %d\n", filename, line, cnt);
            return -1;
        }
        DEBUG("PLACE_MARK '%s' %f %f\n", name, ra, dec);

        strncpy(obj[max_obj].name, name, MAX_OBJ_NAME);
        obj[max_obj].type   = OBJTYPE_PLACE_MARK;;
        obj[max_obj].ra     = ra;
        obj[max_obj].dec    = dec;
        obj[max_obj].mag    = 3;
        obj[max_obj].az     = NO_VALUE;
        obj[max_obj].el     = NO_VALUE;
        obj[max_obj].x      = NO_VALUE;
        obj[max_obj].y      = NO_VALUE;
        obj[max_obj].ssinfo = NULL;
        max_obj++;

        line++;
        num_added++;
    }

    // close
    fclose(fp);

    // success
    INFO("added %d place_mark_objects from %s\n", num_added, filename);
    return 0;
}

// -----------------  PANE HANDLERS  --------------------------------------
// XXX  AAA
// - mouse right click iluminate obj and display info in ctrl pane
// - ctrl pane
//    . info display
//    . mag select
//    . reset (instead of home)
// - clean up

double az_ctr   = 0;
double az_span  = 360;
double el_ctr   = 45;
double el_span  = 90;  
double mag      = DEFAULT_MAG;
int    selected = -1;
bool   tracking = false;

int sky_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event) 
{
    struct {
        int not_used;
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

        int xcoord, ycoord, i, ptsz, color, fontsz, ret;
        double az, el, lst;
        time_t t;
        double grid_sep, first_az_line, first_el_line;

        // reset all az, al, x, y to NO_VALUE
        for (i = 0; i < max_obj; i++) {
            obj_t * x = &obj[i];
            x->x  = NO_VALUE;
            x->y  = NO_VALUE;
            x->az = NO_VALUE;
            x->el = NO_VALUE;
        }

        // get local sidereal time        
        t = time(NULL);
        lst = ct2lst(MY_LONG, jdconv2(t));

        // draw points for objects
        for (i = 0; i < max_obj; i++) {
            obj_t * x = &obj[i];

            // if obj type is not valid then continue
            if (x->type == OBJTYPE_NONE) {
                continue;
            }

            // if processing solar-sys object then compute its current ra, dec, and mag
            if (x->type == OBJTYPE_SOLAR) {
                compute_ss_obj_ra_dec_mag(x, t);
            }

            // magnitude too dim then skip
            if (x->mag > mag) {
                continue;
            }

            // get az/el from ra/dec,
            // sanity check az and el, and
            // convert az to range -180 to + 180
            ret = radec2azel(&az, &el, x->ra, x->dec, lst, MY_LAT);
            if (ret != 0) {
                continue;
            }

            // The azimuth returned from radec2azel is in range 0 to 360; 
            // however the min_az..max_az could be as low as -360 to 0;
            // so, if az is too large, try to correct by reducing it by 360.
            // If az is still out of range min_az..maz_az then the display
            // must be zoomed in, so skip this object.
            if (az > max_az) {
                az -= 360;
            }
            if (az < min_az || az > max_az) {
                continue;
            }

            // determine display point size from object's apparent magnitude
            // XXX try smaaller ptsz, such as '5'
            ptsz = 5 - x->mag;
            if (ptsz < 0) ptsz = 0;
            if (ptsz > 9) ptsz = 9;

            // convert az/el to pane coordinates; 
            // if coords are out of the pane then skip
            ycoord = 0 + k_el * (max_el - el);
            xcoord = 0 + k_az * (min_az - az);
            if (xcoord < 0 || xcoord >= pane->w || ycoord < 0 || ycoord >= pane->h) {
                continue;
            }

            // save the computed object's az/el position, and 
            // pane coordinates
            x->x = xcoord;
            x->y = ycoord;
            x->az = az;  // XXX save orig, not after modified
            x->el = el;

            // render the stellar object point
            color = (i == selected                 ? ORANGE :
                     x->type == OBJTYPE_STELLAR    ? WHITE  :
                     x->type == OBJTYPE_SOLAR      ? YELLOW :
                     x->type == OBJTYPE_PLACE_MARK ? PURPLE :
                                                     BLACK);
            if (color == BLACK) {
                WARN("obj %d '%s' invalid type %d\n", i, x->name, x->type);
                continue;
            }
            sdl_render_point(pane, xcoord, ycoord, color, ptsz);
        }

        // draw grid
        // XXX have problems at the left corners
        grid_sep = az_span > 240 ? 45 :
                   az_span > 120 ? 20 :
                   az_span > 60  ? 10 :
                   az_span > 20  ? 5 :
                   az_span > 8   ? 2 :
                   az_span > 4   ? 1 :
                   az_span > 2   ? 0.5 :
                                   0.25;
        fontsz = 24;
        first_az_line = floor(min_az/grid_sep) * grid_sep;
        for (az = first_az_line; az < max_az; az += grid_sep) {
            double adj_az = (az >= 0 ? az : az + 360);
            char adj_az_str[20];
            int len;
            len = sprintf(adj_az_str, "%g", adj_az);
            xcoord = 0 + k_az * (min_az - az);
            sdl_render_line(pane, xcoord, 0, xcoord, pane->h-1, BLUE);
            sdl_render_printf(pane, xcoord-COL2X(len,fontsz)/2, pane->h-ROW2Y(1,fontsz), fontsz, BLUE, BLACK, "%g", adj_az);
        }
        first_el_line = floor(min_el/grid_sep) * grid_sep;
        for (el = first_el_line; el < max_el; el += grid_sep) {
            ycoord = 0 + k_el * (max_el - el);
            sdl_render_line(pane, 0, ycoord, pane->w-1, ycoord, BLUE);
            sdl_render_printf(pane, 0, ycoord-ROW2Y(1,fontsz)/2, fontsz, BLUE, BLACK, "%g ", el);
        }

        // if zoomed in then display names of the objects that have names
        if (az_span < 100) {
            fontsz = 18;
            for (i = 0; i < max_obj; i++) {
                obj_t * x = &obj[i];
                if (x->name[0] == '\0' || x->x == NO_VALUE || x->y == NO_VALUE) {
                    continue;
                }
                sdl_render_printf(pane, 
                                  x->x+6, x->y-ROW2Y(1,fontsz)/2,
                                  fontsz, WHITE, BLACK, "%s", x->name);
            }
        }

        // XXX
        if (selected != -1 && obj[selected].x == NO_VALUE) {
            selected = -1;
        }
        if (tracking && selected == -1) {
            tracking = false;
        }
        if (tracking) {  // XXX 1 sec delay
            az_ctr = obj[selected].az;
            if (az_ctr > 180) az_ctr -= 360;
            el_ctr = obj[selected].el;
        }

        // register control events 
        rect_t loc = {0,0,pane->w,pane->h};
        sdl_register_event(pane, &loc, SDL_EVENT_MOUSE_MOTION, SDL_EVENT_TYPE_MOUSE_MOTION, pane_cx);
        sdl_register_event(pane, &loc, SDL_EVENT_MOUSE_WHEEL, SDL_EVENT_TYPE_MOUSE_WHEEL, pane_cx);
        sdl_register_event(pane, &loc, SDL_EVENT_MOUSE_RIGHT_CLICK, SDL_EVENT_TYPE_MOUSE_RIGHT_CLICK, pane_cx);

        return PANE_HANDLER_RET_NO_ACTION;
    }

    // -----------------------
    // -------- EVENT --------
    // -----------------------
/** XXX REVIEW
DISPLAY PANE KEYS
- will disable tracking
  - pgup pgdn
  - arrow keys
  - mouse motion
- wont disable tracking
  - mag   m M
  - zoom  z Z
  - mouse wheel  ZOOM
- tbd-xxxx for track on or off
- esc clears selection
- right click enable select and track
TODO
arrow keys
esc  clear select
comments
review
**/

    if (request == PANE_HANDLER_REQ_EVENT) {
        switch (event->event_id) {
        case SDL_EVENT_MOUSE_MOTION: {
            // XXX add arrow keys
            int dx = event->mouse_motion.delta_x;
            int dy = event->mouse_motion.delta_y;
            if (dx == 0 && dy == 0) {
                break;
            }
            az_ctr -= dx * (az_span / 1800.);
            if (az_ctr > 180) az_ctr = 180;
            if (az_ctr < -180) az_ctr = -180;
            el_ctr += dy * (el_span / 450.);
            if (el_ctr > 90) el_ctr = 90;
            if (el_ctr < -90) el_ctr = -90;
            DEBUG("MOUSE MOTION dx=%d dy=%d  az_ctr=%f el_ctr=%f\n", dx, dy, az_ctr, el_ctr);
            tracking = false;
            return PANE_HANDLER_RET_DISPLAY_REDRAW; }
        case SDL_EVENT_MOUSE_WHEEL: case 'z': case 'Z': {
            int dy;

            if (event->event_id == SDL_EVENT_MOUSE_WHEEL) {
                dy = event->mouse_motion.delta_y;
            } else if (event->event_id == 'z') {
                dy = -1;
            } else {
                dy = 1;
            }

            // don't let az_span get abov  360
            if (dy < 0 && az_span * 1.1 > 360) {
                break;
            }
            if (dy < 0) {
                az_span *= 1.1;
                el_span *= 1.1;
            }
            if (dy > 0 && az_span > 1) {
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
                if (x->x == NO_VALUE || x->y == NO_VALUE) {
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
            selected = (best_dist != 999999 ? best_i : -1);
            tracking = false;
            return PANE_HANDLER_RET_DISPLAY_REDRAW; }
        case 27: {  // ESC
            selected = -1;
            tracking = false;
            return PANE_HANDLER_RET_DISPLAY_REDRAW; }
        case SDL_EVENT_KEY_PGUP:
            az_ctr   = 0;
            az_span  = 360;
            el_ctr   = 45;
            el_span  = 90; 
            mag      = DEFAULT_MAG;
            selected = false;
            tracking = false;
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_PGDN:
            az_ctr   = 0;
            az_span  = 360;
            el_ctr   = 0;
            el_span  = 180;
            mag      = DEFAULT_MAG;
            selected = false;
            tracking = false;
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case 'm': case 'M':
            mag += (event->event_id == 'M' ? .1 : -.1);
            if (selected != -1 && obj[selected].mag > mag) {
                selected = -1;
                tracking = false;
            }
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case 'y':
            if (selected != -1) {
                tracking = true;
            }
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case 'n':
            tracking = false;
            return PANE_HANDLER_RET_DISPLAY_REDRAW;


        // XXX select idx can also be cleared with esc key too
#if 0
        case SDL_EVENT_KEY_HOME: {
            return PANE_HANDLER_RET_DISPLAY_REDRAW; }
        case SDL_EVENT_KEY_END:
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

    #define SDL_EVENT_TRACKING_CTL  (SDL_EVENT_USER_DEFINED + 0)

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
        int fontsz=24;
        char name[200];

        sdl_render_text_and_register_event(
            pane, 0, ROW2Y(0,fontsz), fontsz,
            tracking ? "TRACKING" : "NOT_TRACKING",
            LIGHT_BLUE, BLACK, 
            SDL_EVENT_TRACKING_CTL, SDL_EVENT_TYPE_MOUSE_CLICK, pane_cx);

// XXX az display should be consistent with min/maz az
// XXX also need to work on the input line
        sdl_render_printf(pane, 0, ROW2Y(1,fontsz), fontsz, WHITE, BLACK,
                          "AZ/EL  %8.4f %8.4f", az_ctr, el_ctr);
        
        if (selected != -1) {
            obj_t *x = &obj[selected];
            if (x->name[0] != '\0') {
                strcpy(name, x->name);
            } else {
                sprintf(name, "Object-%d", selected);
            }
            sdl_render_printf(pane, 0, ROW2Y(3,fontsz), fontsz, WHITE, BLACK,
                              "%s", name);
            sdl_render_printf(pane, 0, ROW2Y(4,fontsz), fontsz, WHITE, BLACK,
                              "RA/DEC %8.4f %8.4f", x->ra, x->dec);
            sdl_render_printf(pane, 0, ROW2Y(5,fontsz), fontsz, WHITE, BLACK,
                              "AZ/EL  %8.4f %8.4f", x->az, x->el);
            if (x->type != OBJTYPE_PLACE_MARK) {
                sdl_render_printf(pane, 0, ROW2Y(6,fontsz), fontsz, WHITE, BLACK,
                                "MAG   %0.1f", x->mag);
            }
        }

        sdl_render_printf(pane, 0, ROW2Y(8,fontsz), fontsz, WHITE, BLACK,
                          "DISPLAY_MAG   %0.1f", mag);

        return PANE_HANDLER_RET_NO_ACTION;
    }

    // -----------------------
    // -------- EVENT --------
    // -----------------------

    if (request == PANE_HANDLER_REQ_EVENT) {
        switch (event->event_id) {
        case SDL_EVENT_TRACKING_CTL:
            tracking = !tracking && selected != -1;
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        }
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
/**
TRACKING 'OR' NOT_TRACKING     <== can click this, but needs a selection to enable tracking
o AZEL 359.12345 -19.12345

POLARIS                            DONE
o RADEC 244.1234 42.0000            x
o AZEL  xxx       xxxx              x
o MAG   7.1                         x

DISPLAY_MAG  6.0                    x

> <inputs>
sel polaris
sel radec ...   put a place mark at radec
                when a selection is made the display centers and zooms, and track is enabled
sel clear       disables tracking 

track on       
track off

azel xxxx xxxx    disables tracking

mag xxx
zoom xxx
fmt dec
fmt dms
help

FOLLOWED BY STATUS LINE:  OK, ERROR ...
**/


// -----------------  GENERAL UTIL  ---------------------------------------

int compute_ss_obj_ra_dec_mag(obj_t *x, time_t t)
{
    solar_sys_obj_info_t *ssinfo = x->ssinfo;
    struct info_s *info     = ssinfo->info;
    int            max_info = ssinfo->max_info;
    int            idx      = ssinfo->idx_info;
    char           str[100];

    // interpolated_val = v0 + ((v1 - v0) / (t1 - t0)) * (t - t0)
    #define INTERPOLATE(t,v0,v1,t0,t1) \
        ((double)(v0) + (((double)(v1) - (double)(v0)) / ((double)(t1) - (double)(t0))) * ((double)(t) - (double)(t0)))

    if (x->type != OBJTYPE_SOLAR) {
        ERROR("called for invalid obj type %d\n",x->type);
    }

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
    x->ra  = INTERPOLATE(t, info[idx].ra, info[idx+1].ra, info[idx].t, info[idx+1].t); 
    x->dec = INTERPOLATE(t, info[idx].dec, info[idx+1].dec, info[idx].t, info[idx+1].t); 
    x->mag = info[idx].mag;
        
    // save idx hint
    ssinfo->idx_info = idx;

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

// https://idlastro.gsfc.nasa.gov/ftp/pro/astro/jdcnv.pro
// Converts Gregorian dates to Julian days
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
int radec2azel(double *az, double *el, double ra, double dec, double lst, double lat)
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

    // sanity check result
    if (*el < -90 || *el > 90 || *az < 0 || *az > 360) {
        WARN("ra=%f dec=%f cvt to az=%f el=%f, result out of range\n",
             ra, dec, *az, *el);
        return -1;
    }

    // return success
    return 0;
}

// -----------------  TEST ALGORITHMS  ------------------------------------

void test_algorithms(void) 
{
    double jd, lst, ra, dec, az, el, lat, lng;
    double deviation;
    int i, ret;
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
    ret = radec2azel(&az, &el, ra, dec, lst, lat);
    if (ret != 0) {
        ERROR("radec2azel failed\n");
    }
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
    for (i = 0; i < max_obj; i++) {
        obj_t * x = &obj[i];
        if (obj->type == OBJTYPE_SOLAR) {
            compute_ss_obj_ra_dec_mag(x, t);
        }
        ret = radec2azel(&az, &el, x->ra, x->dec, lst, MY_LAT);
        if (ret != 0) {
            ERROR("radec2azel failed\n");
        }
    }
    duration_us = microsec_timer() - start_us;
    INFO("radec2azel perf: %d objects in %ld ms\n", max_obj, duration_us/1000);

    INFO("tests passed\n");
}

bool is_close(double act, double exp, double allowed_deviation, double * deviation)
{
    bool is_close;
    *deviation = fabs((act - exp) / exp);
    is_close = (*deviation < 0.000010);
    return is_close;
}
