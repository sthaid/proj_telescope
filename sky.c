// XXX reveiw and add comments throughout
// XXX organize favorites
// XXX copyright
// XXX README.md
// XXX USAGE
// XXX move NOTES to subdir

#include "common.h"

//
// defines
//

#define MAX_OBJ      200000
#define MAX_OBJ_NAME 32

#define RAD2DEG (180. / M_PI)
#define DEG2RAD (M_PI / 180.)
#define HR2RAD  (M_PI / 12.)

#define NO_VALUE     9999
#define NO_VALUE_STR "9999"

#define OBJTYPE_NONE       0
#define OBJTYPE_STELLAR    1
#define OBJTYPE_SOLAR      2
#define OBJTYPE_PLACE_MARK 3

#define MIN_MAG     -5    // brightest
#define MAX_MAG      22   // dimmest
#define DEFAULT_MAG  7    // faintest naked-eye stars from dark rural area

#define SIZEOF_SOLAR_SYS_OBJ_INFO_T(n) (sizeof(solar_sys_obj_info_t) + sizeof(struct info_s) * (n))

#define SID_DAY_SECS  (23*3600 + 56*60 + 4)

#define SKY_TIME_MODE_NONE    0
#define SKY_TIME_MODE_CURRENT 1
#define SKY_TIME_MODE_PAUSED  2
#define SKY_TIME_MODE_FAST    3
#define SKY_TIME_MODE_SID_DAY 4

#define SKY_TIME_MODE_STR(x) \
    ({ int mode = x; \
       ((mode) == SKY_TIME_MODE_NONE    ? "NONE " : \
        (mode) == SKY_TIME_MODE_CURRENT ? "CURR " : \
        (mode) == SKY_TIME_MODE_PAUSED  ? "PAUSE" : \
        (mode) == SKY_TIME_MODE_FAST    ? "FAST " : \
        (mode) == SKY_TIME_MODE_SID_DAY ? "SDDAY" : \
                                          "????"  );})

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
    } info[0];
} solar_sys_obj_info_t;

typedef struct {
    char name[MAX_OBJ_NAME];
    int type;
    double ra;
    double dec;
    double mag;
    solar_sys_obj_info_t * ssinfo;
    // the following fields are dynamic (utilized during runtime)
    double x;
    double y;
    double az;
    double el;
    double xvp;
    double yvp;
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

// init
int read_stellar_data(char * filename);
int read_solar_sys_data(char * filename);
int read_place_marks(char * filename);
int obj_sanity_checks(void);

// pane support 
char * proc_ctl_pane_cmd(char * cmd_line);

// utils
int compute_ss_obj_ra_dec_mag(obj_t *x, time_t t);
void reset(bool all_sky);
char * gmtime_str(time_t t, char *str);
char *localtime_str(time_t t, char *str);
bool skip_solar_sys_obj(obj_t * x) ;
void sky_time_set_mode(int new_mode_req);
int sky_time_get_mode(void);
time_t sky_time_get_time(void);

// convert ra,dec to az,el
double jdconv(int yr, int mn, int day, double hour);
double jdconv2(time_t t);
double ct2lst(double lng, double jd);
int radec2azel(double *az, double *el, double ra, double dec, double lst, double lat);

// convert az,el to minimally distorted x,y
int azel2xy(double az, double el, double max, double *xret, double *yret);

// unit test
void unit_test_algorithms(void);
bool is_close(double act, double exp, double allowed_deviation, double * deviation);

// -----------------  SKY INIT  -------------------------------------------

int sky_init(void) 
{
    int ret;
    char str[100];

    INFO("UTC now = %s\n", gmtime_str(time(NULL),str));
    INFO("LCL now = %s\n", localtime_str(time(NULL),str));

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

    ret = obj_sanity_checks();
    if (ret < 0) {
        return ret;
    }

    INFO("max_object  = %d\n", max_obj);

    unit_test_algorithms();

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

        // don't know why hygdata_v3.csv has Sol entry at ra=0 dec=0;
        // so just ignore it
        if (strcmp(proper_str, "Sol") == 0) {
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
    int line=1, num_added=0, len;
    obj_t *x = NULL;
    solar_sys_obj_info_t *ssinfo = NULL;
    char str[10000], *s, *name;
    char *date_str, *ra_str, *dec_str, *mag_str;
    char *not_used_str __attribute__ ((unused));
    double ra, dec, mag;
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

            // alloc ssinfo, it will be realloced in increments of 10000 struct info_s as needed
            ssinfo = malloc(SIZEOF_SOLAR_SYS_OBJ_INFO_T(0));
            ssinfo->max_info = 0;
            ssinfo->idx_info = 0;

            // init obj fields
            x = &obj[max_obj];
            strncpy(x->name, name, MAX_OBJ_NAME);
            x->type = OBJTYPE_SOLAR;
            x->ra       = NO_VALUE;
            x->dec      = NO_VALUE;
            x->mag      = NO_VALUE;
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
            mag_str = NO_VALUE_STR;
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
        
        // if the info table for the object currently being input is full
        // then realloc with 10000 more entries
        if ((ssinfo->max_info % 10000) == 0) {
            ssinfo = realloc(ssinfo, SIZEOF_SOLAR_SYS_OBJ_INFO_T(ssinfo->max_info+10000));
            x->ssinfo = ssinfo;
        }

        // fill in the info table for the object currently being input
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
        obj[max_obj].mag    = NO_VALUE;
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

int obj_sanity_checks(void)
{
    int i;

    // verify obj names don't begin or end with a space char
    for (i = 0; i < max_obj; i++) {
        obj_t * x = &obj[i];
        char * name = x->name;
        if (name[0] == '\0') {
            continue;
        }
        if (name[0] == ' ' || name[strlen(name)-1] == ' ') {
            ERROR("obj %d invalid name '%s'\n", i, name);
            return -1;
        }
    }

    // sanity checks all passed
    INFO("obj sanity checks all passed\n");
    return 0;
}

// -----------------  SKY PANE HANDLER  -----------------------------------

time_t sky_time               = 0;
double lst                    = 0;
double az_ctr                 = 0;
double az_span                = 360;
double el_ctr                 = 45;
double el_span                = 90;  
double mag                    = DEFAULT_MAG;
int    selected               = -1;
bool   tracking               = false;
bool   tracking_last          = false;

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

        int xcoord, ycoord, i, ptsz, color, fontsz, ret, ret_action;
        double az, el;
        double grid_sep, first_az_line, first_el_line;

        char str[100];

        // reset all az, al, x, y to NO_VALUE
        for (i = 0; i < max_obj; i++) {
            obj_t * x = &obj[i];
            x->x   = NO_VALUE;
            x->y   = NO_VALUE;
            x->az  = NO_VALUE;
            x->el  = NO_VALUE;
            x->xvp = NO_VALUE;
            x->yvp = NO_VALUE;
        }

        // get sky_time, which can be one of the following:
        // - current time
        // - fast time
        // - advance a siderial day at a time
        sky_time = sky_time_get_time();

        // get local sidereal time        
        lst = ct2lst(longitude, jdconv2(sky_time));

        // draw points for objects
        for (i = 0; i < max_obj; i++) {
            obj_t * x = &obj[i];

            // if obj type is not valid then continue
            if (x->type == OBJTYPE_NONE) {
                continue;
            }

            // this program can be configured to skip certain solar sys objects;
            // to do this the skip_solar_sys_obj routine needs to be adjusted and
            // the program rebuilt
            if (x->type == OBJTYPE_SOLAR && skip_solar_sys_obj(x)) {
                continue;
            }

            // if processing solar-sys object then compute its current ra, dec, and mag
            if (x->type == OBJTYPE_SOLAR) {
                compute_ss_obj_ra_dec_mag(x, sky_time);
            }

            // magnitude too dim then skip; 
            // however don't skip when magnitude is NO_VALUE, such as
            // for place marks or when a solar sys obj has 'n.a.' magnitude
            if (x->mag != NO_VALUE && x->mag > mag) {
                continue;
            }

            // get az/el from ra/dec, and
            // save az/el in obj_t for later use
            ret = radec2azel(&az, &el, x->ra, x->dec, lst, latitude);
            if (ret != 0) {
                continue;
            }
            x->az = az;
            x->el = el;

            // The azimuth returned from radec2azel is in range 0 to 360; 
            // however the min_az..max_az could be as low as -360 to 0;
            // so, if az is too large, try to correct by reducing it by 360.
            if (az > max_az) {
                az -= 360;
            }

            // if az or el out of range then skip
            if (az < min_az || az > max_az || el < min_el || el > max_el) {
                continue;
            }

            // determine display point size from object's apparent magnitude
            ptsz = (x->mag != NO_VALUE ? 5 - x->mag : 3);
            if (ptsz < 0) ptsz = 0;
            if (ptsz > 9) ptsz = 9;

            // convert az/el to pane coordinates; 
            // if coords are out of the pane then skip
            ycoord = 0 + k_el * (max_el - el);
            xcoord = 0 + k_az * (min_az - az);
            if (xcoord < 0 || xcoord >= pane->w || ycoord < 0 || ycoord >= pane->h) {
                continue;
            }

            // save the computed object's pane coords so that they can be used
            // when processing the right-click event, to find the object that is
            // nearest the mouse's right-click x,y pane coordinates
            x->x = xcoord;
            x->y = ycoord;

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
        grid_sep = az_span > 240 ? 45 :
                   az_span > 120 ? 20 :
                   az_span > 60  ? 10 :
                   az_span > 25  ? 5 :
                   az_span > 10  ? 2 :
                   az_span > 6   ? 1 :
                                   0.5;
        fontsz = 24;
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

        // if zoomed in then display names of the objects that have names
        fontsz = 18;
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

        // display date & time
        fontsz = 24;
        color = (sky_time_get_mode() == SKY_TIME_MODE_CURRENT ? WHITE : RED);
        sdl_render_printf(pane, COL2X(3,fontsz), 0, fontsz, color, BLACK, "%s %s", 
                          SKY_TIME_MODE_STR(sky_time_get_mode()),
                          localtime_str(sky_time,str));

        // clear selected if the magnitude of the selected obj is no longer being displayed
        if (selected != -1 && obj[selected].mag != NO_VALUE && obj[selected].mag > mag) {
            selected = -1;
        }

        // if we're tracking the selected obj, and there is no longer a selected obj
        // then turn tracking off
        if (tracking && selected == -1) {
            tracking = false;
        }

        // if we're tracking then update the az/el of sky_pane center to the
        // az/el of the selected object
        if (tracking) {
            radec2azel(&az_ctr, &el_ctr, obj[selected].ra, obj[selected].dec, lst, latitude);
            if (az_ctr > 180) az_ctr -= 360;
        }

        // if tracking has been enabled then do an immediae display redraw
        ret_action = (tracking && !tracking_last 
                      ? PANE_HANDLER_RET_DISPLAY_REDRAW
                      : PANE_HANDLER_RET_NO_ACTION);
        tracking_last = tracking;

        // register control events 
        rect_t loc = {0,0,pane->w,pane->h};
        sdl_register_event(pane, &loc, SDL_EVENT_MOUSE_MOTION, SDL_EVENT_TYPE_MOUSE_MOTION, pane_cx);
        sdl_register_event(pane, &loc, SDL_EVENT_MOUSE_WHEEL, SDL_EVENT_TYPE_MOUSE_WHEEL, pane_cx);
        sdl_register_event(pane, &loc, SDL_EVENT_MOUSE_RIGHT_CLICK, SDL_EVENT_TYPE_MOUSE_RIGHT_CLICK, pane_cx);

        return ret_action;;
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
                dx = 5;
            } else if (event->event_id == SDL_EVENT_KEY_UP_ARROW) {
                dy = -5;
            } else if (event->event_id == SDL_EVENT_KEY_DOWN_ARROW) {
                dy = 5;
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
        case 'm': case 'M':
            mag += (event->event_id == 'M' ? .1 : -.1);
            if (mag < MIN_MAG) mag = MIN_MAG;
            if (mag > MAX_MAG) mag = MAX_MAG;
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
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
            selected = (best_dist != 999999 ? best_i : -1);
            tracking = false;
            return PANE_HANDLER_RET_DISPLAY_REDRAW; }
        case 27: {  // ESC
            selected = -1;
            tracking = false;
            return PANE_HANDLER_RET_DISPLAY_REDRAW; }
        case 't': case 'T':
            tracking = (event->event_id == 'T' && selected != -1);
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_PGUP:
            reset(false);
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_PGDN:
            reset(true);
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case '1':
            sky_time_set_mode(SKY_TIME_MODE_CURRENT);
            break;
        case '2':
            sky_time_set_mode(SKY_TIME_MODE_PAUSED);
            break;
        case '3':
            sky_time_set_mode(SKY_TIME_MODE_FAST);
            break;
        case '4':
            sky_time_set_mode(SKY_TIME_MODE_SID_DAY);
            break;
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

// -----------------  SKY CONTROL PANE HANDLER  ---------------------------

int sky_ctl_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event) 
{
    struct {
        char   cmd_line[1000];
        char   cmd_line_last[1000];
        int    cmd_line_len;
        char * cmd_status;
        long   cmd_status_time_us;
    } * vars = pane_cx->vars;
    rect_t * pane = &pane_cx->pane;

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
        int fontsz=24, ret;
        char name[200];
        double az, el, az_ctr1;

        sdl_render_printf(pane, 0, ROW2Y(0,fontsz), fontsz, WHITE, BLACK, 
                          tracking ? "TRACKING" : "NOT_TRACKING");

        az_ctr1 = (az_ctr >= 0 ? az_ctr : az_ctr + 360);
        sdl_render_printf(pane, 0, ROW2Y(1,fontsz), fontsz, WHITE, BLACK,
                          "AZ/EL  %8.4f %8.4f", az_ctr1, el_ctr);
        
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
            ret = radec2azel(&az, &el, x->ra, x->dec, lst, latitude);
            if (ret == 0) {
                sdl_render_printf(pane, 0, ROW2Y(5,fontsz), fontsz, WHITE, BLACK,
                                  "AZ/EL  %8.4f %8.4f", az, el);
            }
            if (x->mag != NO_VALUE) {
                sdl_render_printf(pane, 0, ROW2Y(6,fontsz), fontsz, WHITE, BLACK,
                                  "MAG   %0.1f", x->mag);
            }
        }

        sdl_render_printf(pane, 0, ROW2Y(8,fontsz), fontsz, WHITE, BLACK,
                          "DISPLAY_MAG   %0.1f", mag);

        if (microsec_timer() - vars->cmd_status_time_us < 2500000) {
            sdl_render_printf(pane, 0, ROW2Y(10,fontsz), fontsz, WHITE, BLACK,
                              "> %s", vars->cmd_line_last);
            sdl_render_printf(pane, 0, ROW2Y(11,fontsz), fontsz, WHITE, BLACK,
                              "  %s", vars->cmd_status);
        } else {
            sdl_render_printf(pane, 0, ROW2Y(10,fontsz), fontsz, WHITE, BLACK,
                              "> %s%c", vars->cmd_line, '_');
        }

        return PANE_HANDLER_RET_NO_ACTION;
    }

    // -----------------------
    // -------- EVENT --------
    // -----------------------

    if (request == PANE_HANDLER_REQ_EVENT) {
        switch (event->event_id) {
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
                    vars->cmd_status = proc_ctl_pane_cmd(vars->cmd_line);
                    vars->cmd_status_time_us = microsec_timer();
                    strcpy(vars->cmd_line_last, vars->cmd_line);
                    vars->cmd_line_len = 0;
                    vars->cmd_line[0]  = '\0';          
                }
            }
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

char * proc_ctl_pane_cmd(char * cmd_line)
{
    int i;
    char *cmd, *arg1;
    char cmd_line_copy[1000];

    // tokenize cmd_line into cmd, arg1
    strcpy(cmd_line_copy, cmd_line);
    cmd = strtok(cmd_line_copy, " ");
    arg1 = strtok(NULL, " ");

    // ignore blank cmd_line
    if (cmd == NULL) {
        return "";
    }

    if (strcasecmp(cmd, "sel") == 0) {
        // process: sel [<object_name>]
        selected = -1;
        if (arg1 == NULL) {
            return "okay";
        }
        for (i = 0; i < max_obj; i++) {
            obj_t *x = &obj[i];
            if (strcasecmp(arg1, x->name) == 0) {
                if (obj[i].mag != NO_VALUE && obj[i].mag > mag) {
                    return "error: obj too dim";
                }
                selected = i;
                tracking = false;
                return "okay";
            }
        }
        return "error: not found";
    } else if (strcasecmp(cmd, "trk") == 0) {
        // process: trk <on|off>
        if ((arg1 != NULL) && (strcasecmp(arg1, "on") && strcasecmp(arg1, "off"))) {
            return "error: expected 'on', 'off'";
        }
        if (arg1 == NULL || strcasecmp(arg1, "on") == 0) {
            if (selected == -1) {
                return "error: no selection";
            }
            tracking = true;
        } else {
            tracking = false;
        }
        return "okay";
    } else if (strcasecmp(cmd, "reset") == 0) {
        // process: reset [<all_sky>]
        if (arg1 != NULL && strcasecmp(arg1, "all_sky")) {
            return "error: expected '', 'all_sky'";
        }
        reset(arg1 ? true : false);
        return "okay";
    } else if (strcasecmp(cmd, "zoom") == 0) {
        int n;
        // process: zoom <1..52>
        if (arg1 == NULL || sscanf(arg1, "%d", &n) != 1 || n < 1 || n > 52) {
            return "error: expected 1..52";
        }
        if (az_span/el_span > 3.9999) {
            az_span  = 360;
            el_span  = 90;
        } else {
            az_span  = 360;
            el_span  = 180;
        }
        for (i = 1; i < n; i++) {
            az_span /= 1.1;
            el_span /= 1.1;
        }
        return "okay";
    } else if (strcasecmp(cmd, "mag") == 0) {
        // process 'mag' cmd
        if (arg1 == NULL) {
            mag = DEFAULT_MAG;
        } else {
            int new_mag;
            if (sscanf(arg1, "%d", &new_mag) != 1 || new_mag < MIN_MAG || new_mag > MAX_MAG) {
                return "error: invalid mag";
            }
            mag = new_mag;
        }
        return "okay";
    } else if (strcasecmp(cmd, "quit") == 0) {
        exit(0);
        // not reached
    }

    return "error: invalid command";
}

// -----------------  SKY VIEW PANE HANDLER  ------------------------------

int    sky_view_scale_tbl_idx = 0;
double sky_view_scale_tbl[]   = {45, 40, 35, 30, 25, 20, 15, 10, 5, 4, 3, 2, 1, 0.5};

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
        INFO("PANE x,y,w,h  %d %d %d %d\n",
            pane->x, pane->y, pane->w, pane->h);
        return PANE_HANDLER_RET_NO_ACTION;
    }

    // ------------------------
    // -------- RENDER --------
    // ------------------------

    if (request == PANE_HANDLER_REQ_RENDER) {
        rect_t * pane = &pane_cx->pane;
        int i, ret, ptsz, color, xcoord, ycoord;
        double xret, yret, max;
        int fontsz=24;

        for (i = 0; i < max_obj; i++) {
            obj_t * x = &obj[i];

            // if obj type is not valid then continue
            if (x->type == OBJTYPE_NONE) {
                continue;
            }

            // this program can be configured to skip certain solar sys objects;
            // to do this the skip_solar_sys_obj routine needs to be adjusted and
            // the program rebuilt
            if (x->type == OBJTYPE_SOLAR && skip_solar_sys_obj(x)) {
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

        // draw cross hair
        sdl_render_line(pane,
                        pane->w/2-20, pane->h/2, pane->w/2+20, pane->h/2,
                        RED);
        sdl_render_line(pane,
                        pane->w/2, pane->h/2-20, pane->w/2, pane->h/2+20,
                        RED);

        // if zoomed in then display names of the objects that have names
        fontsz = 18;
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
                dx = 5;
            } else if (event->event_id == SDL_EVENT_KEY_UP_ARROW) {
                dy = -5;
            } else if (event->event_id == SDL_EVENT_KEY_DOWN_ARROW) {
                dy = 5;
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

            DEBUG("MOUSE MOTION dx=%d dy=%d  az_ctr=%f el_ctr=%f\n", dx, dy, az_ctr, el_ctr);

            tracking = false;
            return PANE_HANDLER_RET_DISPLAY_REDRAW; }
        case SDL_EVENT_MOUSE_WHEEL: case 'z': case 'Z': {
            if (event->event_id == SDL_EVENT_MOUSE_WHEEL) {
                if (event->mouse_motion.delta_y > 0) {
                    sky_view_scale_tbl_idx++;
                } else if (event->mouse_motion.delta_y < 0) {
                    sky_view_scale_tbl_idx--;
                }
            } else if (event->event_id == 'z') {
                sky_view_scale_tbl_idx--;
            } else {
                sky_view_scale_tbl_idx++;
            }

            if (sky_view_scale_tbl_idx < 0) sky_view_scale_tbl_idx = 0;
            if (sky_view_scale_tbl_idx >= MAX_SCALE_TBL) sky_view_scale_tbl_idx = MAX_SCALE_TBL-1;
            DEBUG("MOUSE WHEEL sky_view_scale_tbl_idx = %d\n", sky_view_scale_tbl_idx);
            return PANE_HANDLER_RET_DISPLAY_REDRAW; }
        case 'm': case 'M':
            mag += (event->event_id == 'M' ? .1 : -.1);
            if (mag < MIN_MAG) mag = MIN_MAG;
            if (mag > MAX_MAG) mag = MAX_MAG;
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_PGUP:
            reset(false);
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case SDL_EVENT_KEY_PGDN:
            reset(true);
            return PANE_HANDLER_RET_DISPLAY_REDRAW;
        case '1':
            sky_time_set_mode(SKY_TIME_MODE_CURRENT);
            break;
        case '2':
            sky_time_set_mode(SKY_TIME_MODE_PAUSED);
            break;
        case '3':
            sky_time_set_mode(SKY_TIME_MODE_FAST);
            break;
        case '4':
            sky_time_set_mode(SKY_TIME_MODE_SID_DAY);
            break;
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
        FATAL("called for invalid obj type %d\n",x->type);
    }

    if (t >= info[idx].t && t <= info[idx+1].t) {
        goto interpolate;
    }

    while (t < info[idx].t) {
        idx--;
        if (idx < 0) {
            FATAL("time %s too early for solar_sys_object %s\n", gmtime_str(t,str), x->name);
        }
    }

    while (t > info[idx+1].t) {
        idx++;
        if (idx > max_info-2) {
            FATAL("time %s too large for solar_sys_object %s\n", gmtime_str(t,str), x->name);
        }
    }

    if (t < info[idx].t || t > info[idx+1].t) {
        FATAL("bug t=%ld info[%d].t=%ld info[%d].t=%ld\n",
              t, idx, info[idx].t, idx+1, info[idx+1].t);
    }

interpolate:
    // determine ra and dec return values by interpolation;
    // don't interpolating mag, it can have NO_VALUE
    x->ra  = INTERPOLATE(t, info[idx].ra, info[idx+1].ra, info[idx].t, info[idx+1].t); 
    x->dec = INTERPOLATE(t, info[idx].dec, info[idx+1].dec, info[idx].t, info[idx+1].t); 
    x->mag = info[idx].mag;
        
    // save idx hint
    ssinfo->idx_info = idx;

    // return success
    return 0;
}

void reset(bool all_sky) 
{
    az_ctr   = 0;
    az_span  = 360;
    if (all_sky) {
        el_ctr   = 0;
        el_span  = 180;
    } else {
        el_ctr   = 45;
        el_span  = 90; 
    }
    mag = DEFAULT_MAG;
    tracking = false;
    sky_view_scale_tbl_idx = 0;
    sky_time_set_mode(SKY_TIME_MODE_CURRENT);
}

char *gmtime_str(time_t t, char *str)
{
    struct tm *tm;

    tm = gmtime(&t);
    sprintf(str, "%4d-%s-%2.2d %2.2d:%2.2d:%2.2d UTC",
            tm->tm_year+1900, 
            month_tbl[tm->tm_mon],
            tm->tm_mday,
            tm->tm_hour,
            tm->tm_min,
            tm->tm_sec);
    return str;
}

char *localtime_str(time_t t, char *str)
{
    struct tm *tm;

    tm = localtime(&t);
    sprintf(str, "%4d-%s-%2.2d %2.2d:%2.2d:%2.2d %s",
            tm->tm_year+1900, 
            month_tbl[tm->tm_mon],
            tm->tm_mday,
            tm->tm_hour,
            tm->tm_min,
            tm->tm_sec,
            tzname[tm->tm_isdst]);
    return str;
}

bool skip_solar_sys_obj(obj_t * x) 
{
    if (x->type != OBJTYPE_SOLAR) {
        return false;
    }

    // uncomment the solar sys objects you want to skip

#if 0
    if (strcmp(x->name, "Sun")     == 0) return true;
    if (strcmp(x->name, "Mercury") == 0) return true;
    if (strcmp(x->name, "Venus")   == 0) return true;
    if (strcmp(x->name, "Mars")    == 0) return true;
    if (strcmp(x->name, "Jupiter") == 0) return true;
    if (strcmp(x->name, "Saturn")  == 0) return true;
    if (strcmp(x->name, "Uranus")  == 0) return true;
    if (strcmp(x->name, "Neptune") == 0) return true;
    if (strcmp(x->name, "Pluto")   == 0) return true;
    if (strcmp(x->name, "Moon")    == 0) return true;
#else  // vvvvv skip these vvvvv
#endif

    return false;
}

time_t __sky_time                  = 0;
long   __sky_time_new_mode_req     = SKY_TIME_MODE_NONE;
long   __sky_time_mode             = SKY_TIME_MODE_NONE;
time_t __sky_time_mode_entry_time  = 0;
long   __sky_time_mode_count    = 0;

void sky_time_set_mode(int new_mode_req)
{
    __sky_time_new_mode_req = new_mode_req;
}

int sky_time_get_mode(void)
{
    return __sky_time_mode;
}

time_t sky_time_get_time(void) 
{
    if (__sky_time == 0) {
        __sky_time = time(NULL);
        __sky_time_new_mode_req = SKY_TIME_MODE_CURRENT;
    }

    // XXX comment this vvvv  OR could get this from the last time in solar sys data
    if (__sky_time > 1640834822 && __sky_time_new_mode_req != SKY_TIME_MODE_CURRENT) {
        __sky_time_new_mode_req = SKY_TIME_MODE_PAUSED;
    }

    if (__sky_time_new_mode_req != SKY_TIME_MODE_NONE) {
        __sky_time_mode = __sky_time_new_mode_req;
        __sky_time_mode_count = 0;
        __sky_time_mode_entry_time = __sky_time;
        __sky_time_new_mode_req = SKY_TIME_MODE_NONE;
    }
    __sky_time_mode_count++;

    switch (__sky_time_mode) {
    case SKY_TIME_MODE_CURRENT:
        __sky_time = time(NULL);
        break;
    case SKY_TIME_MODE_PAUSED:
        // no change to __sky_time.
        break;
    case SKY_TIME_MODE_FAST:
        __sky_time = __sky_time_mode_entry_time +
                     __sky_time_mode_count * 100;
        break;
    case SKY_TIME_MODE_SID_DAY:
        __sky_time = __sky_time_mode_entry_time +
                     __sky_time_mode_count * SID_DAY_SECS;
        break;
    default:
        FATAL("invalid sky_time_mode %ld\n", __sky_time_mode);
    }

    DEBUG("sky_time=%ld mode=%s mode_entry_time=%ld mode_count=%ld \n",
         __sky_time, SKY_TIME_MODE_STR(__sky_time_mode), 
         __sky_time_mode_entry_time, __sky_time_mode_count);

    return __sky_time;
} 

// -----------------  CONVERT RA/DEC TO AZ/EL UTILS  ----------------------

// INFO
// - https://en.wikipedia.org/wiki/Epoch_(reference_date)#J2000.0
//   The current standard epoch is called "J2000.0" This is defined by 
//   international agreement to be equivalent to:
//  . The Gregorian date January 1, 2000 at approximately 12:00 GMT (Greenwich Mean Time).
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

// -----------------  CONVERT DELTA FROM AZ/EL TO AZ_CTR/EL_CTR -----------
// -----------------       TO MINIMAL DISTORTED XRET/YRET       -----------

// This routine converts the position of an object (az/el) from the sky_pane
// center az_ctr/el_ctr to xret/yret which have minimal distortion; the xret/yret
// range from -1 to +1, where a value of -1 or +1 is equivalent to the max arg.
//
// For example: 
//   inputs: az_ctr=90  el_ctr=0
//           az=95  el=0  max=5
//   outputs: xret=1  yret=0
//
// This routine returns 
//   0 on success, or 
//  -1 if the az/el deviation from az_ctr/el_ctr by more than max

int azel2xy(double az, double el, double max, double *xret, double *yret)
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

// -----------------  TEST ALGORITHMS  ------------------------------------

void unit_test_algorithms(void) 
{
    double jd, lst, ra, dec, az, el, lat, lng, az_exp, el_exp;
    double deviation;
    int i, ret;
    unsigned long start_us, duration_us;
    time_t t;

    // test jdconv ...
    // https://www.aavso.org/jd-calculator; also
    // refer to https://en.wikipedia.org/wiki/Epoch_(astronomy)#Julian_years_and_J2000
    jd =  jdconv(2000,1,1,12);
    if (jd != 2451545.0) {
        FATAL("jd %f should be 2451545.0\n", jd);
    }

    // test local sidereal time ...
    // from date cmd: Fri Dec  7 20:53:37 EST 2018
    // from https://tycho.usno.navy.mil/cgi-bin/sidereal-post.sh
    //    02:12:44.9 LST         2+12/60+44.9/3600 = 2.21247
    //    Longitude -72 00 00
    jd = jdconv(2018,12,8,1.8936);  
    lst = ct2lst(-72, jd);
    if (!is_close(lst, 2.21247, 0.000010, &deviation)) {
        FATAL("lst deviation = %f, exceeds limit\n", deviation);
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
    az_exp = 311.92258;
    el_exp = 22.40100;
    ret = radec2azel(&az, &el, ra, dec, lst, lat);
    if (ret != 0) {
        FATAL("radec2azel failed\n");
    }
    DEBUG("calc: az=%f el=%f  expected: az=%f el=%f\n", az, el, az_exp, el_exp);
    if (!is_close(az, az_exp, 0.000010, &deviation)) {
        INFO("az deviation = %f, exceeds limit\n", deviation);
    }
    INFO("az deviation = %f\n", deviation);
    if (!is_close(el, el_exp, 0.000010, &deviation)) {
        INFO("el deviation = %f, exceeds limit\n", deviation);
    }
    INFO("el deviation = %f\n", deviation);

    // print list of solar_sys objects and their ra,dec,mag,az,el
    t = time(NULL);
    lst = ct2lst(longitude, jdconv2(t));
    INFO("            NAME         RA        DEC        MAG         AZ         EL\n");
    for (i = 0; i < max_obj; i++) {
        obj_t *x = &obj[i];
        double az, el;

        if (x->type != OBJTYPE_SOLAR) {
            continue;
        }

        compute_ss_obj_ra_dec_mag(x, t);
        ret = radec2azel(&az, &el, x->ra, x->dec, lst, latitude);
        if (ret != 0) {
            FATAL("radec2azel failed\n");
        }

        INFO("%16s %10.4f %10.4f %10.1f %10.4f %10.4f\n", x->name, x->ra, x->dec, x->mag, az, el);
    }

    // time how long to convert all stellar objects to az/el
    start_us = microsec_timer();
    t = time(NULL);
    lst = ct2lst(longitude, jdconv2(t));
    for (i = 0; i < max_obj; i++) {
        obj_t * x = &obj[i];
        if (obj->type == OBJTYPE_SOLAR) {
            compute_ss_obj_ra_dec_mag(x, t);
        }
        ret = radec2azel(&az, &el, x->ra, x->dec, lst, latitude);
        if (ret != 0) {
            FATAL("radec2azel failed\n");
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
