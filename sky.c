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

//
// defines
//

#define MAX_OBJ      200000
#define MAX_OBJ_NAME 32

#define OBJTYPE_NONE       0
#define OBJTYPE_STELLAR    1
#define OBJTYPE_SOLAR      2
#define OBJTYPE_PLACE_MARK 3

#define SIZEOF_SOLAR_SYS_OBJ_INFO_T(n) (sizeof(solar_sys_obj_info_t) + sizeof(struct info_s) * (n))

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

#define SIND(x)   (sin((x)*DEG2RAD))
#define COSD(x)   (cos((x)*DEG2RAD))
#define TAND(x)   (tan((x)*DEG2RAD))
#define ACOSD(x)  (acos(x)*RAD2DEG)
#define ASIND(x)  (asin(x)*RAD2DEG)

#define RAD2DEG (180. / M_PI)
#define DEG2RAD (M_PI / 180.)
#define HR2RAD  (M_PI / 12.)

#define MIN_MAG     -5    // brightest
#define MAX_MAG      22   // dimmest
#define DEFAULT_MAG  7    // faintest naked-eye stars from dark rural area

#define SID_DAY_SECS  (23*3600 + 56*60 + 4)

#define JD2000 2451545.0

#define DELTA_T 180

#define NO_VALUE     9999
#define NO_VALUE_STR "9999"

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
int read_solar_sys_data(char * filename, char **incl_ss_obj, int max_incl_ss_obj);
int read_place_marks(char * filename);
int obj_sanity_checks(void);

// pane support 
char * proc_ctl_pane_cmd(char * cmd_line);

// utils
int compute_ss_obj_ra_dec_mag(obj_t *x, time_t t);
void reset(bool all_sky);
char * gmtime_str(time_t t, char *str);
char *localtime_str(time_t t, char *str);
void hr2hms(double hr, int * hour, int * minute, double * seconds);
char * hr_str(double hr, char *str);
void get_next_day(int * year, int * month, int * day);
void get_prior_day(int * year, int * month, int * day);
void sky_time_set_mode(int mode);
void sky_time_get_mode(int * mode);
void sky_time_set_step_mode(int step_mode, double step_mode_hr_param);
void sky_time_get_step_mode(int *step_mode, double *step_mode_hr_param);
time_t sky_time_get_time(void);
time_t sky_time_tod_next(time_t t, double hr);
time_t sky_time_tod_prior(time_t t, double hr);

// convert ra,dec to az,el
double jdconv(int yr, int mn, int day, double hour);
double jdconv2(time_t t);
void jd2ymdh(double jd, int *year, int *month, int *day, double *hour);
double ct2lst(double lng, double jd);
int radec2azel(double *az, double *el, double ra, double dec, double lst, double lat);

// convert az,el to minimally distorted x,y
int azel2xy(double az, double el, double max, double *xret, double *yret);

// sunrise and sunset times
void sunrise_sunset(double jd, time_t *trise, time_t *tset);

// unit test
void unit_test(void);
bool is_close(double act, double exp, double allowed_deviation, double * deviation);

// -----------------  SKY INIT  -------------------------------------------

int sky_init(char *incl_ss_obj_str)
{
    int ret;
    char str[100];
    bool first = true;
    char *incl_ss_obj[100];
    int max_incl_ss_obj = 0;

    INFO("UTC now = %s\n", gmtime_str(time(NULL),str));
    INFO("LCL now = %s\n", localtime_str(time(NULL),str));

    // parse incl_ss_obj_str, which is a comma seperated list of 
    // solar sys objects to be included; if the list is not supplied
    // then all solar sys objects will be included
    if (incl_ss_obj_str) {
        while (true) {
            char *name = strtok(first ? incl_ss_obj_str : NULL, ",");
            first = false;
            if (name == NULL) {
                break;
            }
            incl_ss_obj[max_incl_ss_obj++] = name;
        }
    }

    // read positions of stars from hygdata_v3.csv"
    ret = read_stellar_data("sky_data/hygdata_v3.csv");
    if (ret < 0) {
        return ret;
    }

    // read positions of solar sys objects (planets, moons, etc) from solar_sys_data.csv
    ret = read_solar_sys_data("sky_data/solar_sys_data.csv", incl_ss_obj, max_incl_ss_obj);
    if (ret < 0) {
        return ret;
    }

    // read additional place markers from place_marks.dat
    ret = read_place_marks("sky_data/place_marks.dat");
    if (ret < 0) {
        return ret;
    }

    // do some sanity checks on the objects that have been read by the calls made above
    ret = obj_sanity_checks();
    if (ret < 0) {
        return ret;
    }

    // debug print the total number of objects that have been read
    INFO("max_object  = %d\n", max_obj);

    // run some unit tests (optional)
    unit_test();

    // success
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

        DEBUG("%16s %10.4f %10.4f %10.4f\n", proper_str, ra, dec, mag);
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

int read_solar_sys_data(char *filename, char **incl_ss_obj, int max_incl_ss_obj)
{
    // format, example:
    //   # Venus
    //    2018-Dec-01 00:00, , ,207.30643, -9.79665,  -4.87,  1.44,

    FILE *fp;
    int line=1, num_added=0, len, i;
    obj_t *x = NULL;
    solar_sys_obj_info_t *ssinfo = NULL;
    char str[10000], *s, *name;
    char *date_str, *ra_str, *dec_str, *mag_str;
    char *not_used_str __attribute__ ((unused));
    double ra, dec, mag;
    time_t t;
    bool skipping_this_ss_obj=false;

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

            // if a list of solar sys objects to include has been provided then
            // check the list for presenced of 'name'; if not found then this object
            // will be skipped
            if (max_incl_ss_obj > 0) {
                skipping_this_ss_obj = true;
                for (i = 0; i < max_incl_ss_obj; i++) {
                    if (strcasecmp(name, incl_ss_obj[i]) == 0) {
                        skipping_this_ss_obj = false;
                        break;
                    }
                }
            } else {
                skipping_this_ss_obj = false;
            }
            if (skipping_this_ss_obj) {
                continue;
            }

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

        // if skipping this object then continue
        if (skipping_this_ss_obj) {
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

    // success
    INFO("added %d solar_sys_objects from %s\n", num_added, filename);
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
    DEBUG("obj sanity checks all passed\n");
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
        DEBUG("PANE x,y,w,h  %d %d %d %d\n",
              pane->x, pane->y, pane->w, pane->h);
        return PANE_HANDLER_RET_NO_ACTION;
    }

    // ------------------------
    // -------- RENDER --------
    // ------------------------

    if (request == PANE_HANDLER_REQ_RENDER) {
        double min_az = az_ctr - az_span / 2.;
        double max_az = az_ctr + az_span / 2.;
        double min_el = el_ctr - el_span / 2.;
        double max_el = el_ctr + el_span / 2.;
        double k_el = (pane->h) / (max_el - min_el);
        double k_az = (pane->w) / (min_az - max_az);

        int xcoord, ycoord, i, ptsz, color, fontsz, ret, ret_action, mode;
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
        // - pause
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

            // if processing solar-sys object then compute its current ra, dec, and mag
            if (x->type == OBJTYPE_SOLAR) {
                if (compute_ss_obj_ra_dec_mag(x, sky_time) != 0) {
                    continue;
                }
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

        // display names of the objects that have names
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
        // XXX not green on single step
        fontsz = 24;
        sky_time_get_mode(&mode);
        color = (mode == SKY_TIME_MODE_CURRENT ? WHITE :
                 mode == SKY_TIME_MODE_PAUSED  ? RED   :
                                                 GREEN);
        sdl_render_printf(pane, COL2X(3,fontsz), 0, fontsz, color, BLACK, "%8s %s", 
                          SKY_TIME_MODE_STR(mode),
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

            tracking = false;

            DEBUG("MOUSE MOTION dx=%d dy=%d  az_ctr=%f el_ctr=%f\n", dx, dy, az_ctr, el_ctr);
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
            sky_time_set_mode(SKY_TIME_MODE_REV);
            break;
        case '4':
            sky_time_set_mode(SKY_TIME_MODE_FWD);
            break;
        case '5':
            sky_time_set_mode(SKY_TIME_MODE_REV_STEP);
            break;
        case '6':
            sky_time_set_mode(SKY_TIME_MODE_FWD_STEP);
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
        DEBUG("PANE x,y,w,h  %d %d %d %d\n",
            pane->x, pane->y, pane->w, pane->h);
        return PANE_HANDLER_RET_NO_ACTION;
    }

    // ------------------------
    // -------- RENDER --------
    // ------------------------

    if (request == PANE_HANDLER_REQ_RENDER) {
        int fontsz=24, ret, step_mode;
        char name[200];
        double az, el, az_ctr1, step_mode_hr_param;

        // display tracking state
        sdl_render_printf(pane, 0, ROW2Y(0,fontsz), fontsz, WHITE, BLACK, 
                          tracking ? "TRACKING" : "NOT_TRACKING");

        // display az/el of the center the sky az/el, and sky view pane
        az_ctr1 = (az_ctr >= 0 ? az_ctr : az_ctr + 360);
        sdl_render_printf(pane, 0, ROW2Y(1,fontsz), fontsz, WHITE, BLACK,
                          "AZ/EL  %8.4f %8.4f", az_ctr1, el_ctr);
        
        // if there is a selected object, then display it's info
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

        // display the minimum object magnitude that will be displayed
        sdl_render_printf(pane, 0, ROW2Y(8,fontsz), fontsz, WHITE, BLACK,
                          "MAG:   %0.1f", mag);

        // display the time step mode setting
        sky_time_get_step_mode(&step_mode, &step_mode_hr_param);
        if (step_mode_hr_param == 0) {
            sdl_render_printf(pane, 0, ROW2Y(9,fontsz), fontsz, WHITE, BLACK,
                            "TSTEP: %s", SKY_TIME_STEP_MODE_STR(step_mode));
        } else {
            sdl_render_printf(pane, 0, ROW2Y(9,fontsz), fontsz, WHITE, BLACK,
                            "TSTEP: %s%+0.3g", SKY_TIME_STEP_MODE_STR(step_mode), step_mode_hr_param);
        }

        // display the cmd_line and cmd_status
        if (microsec_timer() - vars->cmd_status_time_us < 2500000) {
            sdl_render_printf(pane, 0, ROW2Y(11,fontsz), fontsz, WHITE, BLACK,
                              "> %s", vars->cmd_line_last);
            sdl_render_printf(pane, 0, ROW2Y(12,fontsz), fontsz, WHITE, BLACK,
                              "  %s", vars->cmd_status);
        } else {
            sdl_render_printf(pane, 0, ROW2Y(11,fontsz), fontsz, WHITE, BLACK,
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
    arg1 = strtok(NULL, "");
    INFO("cmd='%s' arg1='%s'\n", cmd, arg1);

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
        // process: mag <mag> 
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
    } else if (strcasecmp(cmd, "tstep") == 0) {
        // process: tstep <delta_t|sunrise[+/-h.hh]|sunset[+/-h.hh]|sidday|h.hhh>
        //
        // This command sets the time-mode that is used when SKY_TIME_MODE_REV/FWD
        // is enabled. SKY_TIME_MODE_xxx is controlled using the key cmds 1,2,3,4,5,6
        // in the sky pane or sky view pane.
        // 
        // EXAMPLES          TIME OF NEXT DISPLAY UPDATE IN SKY_TIME_MODE_FWD
        // --------           ------------------------------------------------
        // tstep delta_t      current time + 120 secs
        // tstep sunset       next day sunset
        // tstep sunset+2.5   next day sunset + 2 1/2 hours
        // tstep sidday       current time + SID_DAY_SECS
        // tstep 23.25        23:15:00 UTC of the next day 

        double hr = 0;
        if (arg1 == NULL) {
            return "error: arg required";
        }

        if (strcasecmp(arg1, "delta_t") == 0) {
            sky_time_set_step_mode(SKY_TIME_STEP_MODE_DELTA_T, 0);
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
                if (sscanf(arg1+6, "%lf", &hr) != 1) {
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
        DEBUG("PANE x,y,w,h  %d %d %d %d\n",
            pane->x, pane->y, pane->w, pane->h);
        return PANE_HANDLER_RET_NO_ACTION;
    }

    // ------------------------
    // -------- RENDER --------
    // ------------------------

    if (request == PANE_HANDLER_REQ_RENDER) {
        int i, ret, ptsz, color, xcoord, ycoord;
        double xret, yret, max;
        int fontsz=24;

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

        // display names of the objects that have names
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

            tracking = false;

            DEBUG("MOUSE MOTION dx=%d dy=%d  az_ctr=%f el_ctr=%f\n", dx, dy, az_ctr, el_ctr);
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
            sky_time_set_mode(SKY_TIME_MODE_REV);
            break;
        case '4':
            sky_time_set_mode(SKY_TIME_MODE_FWD);
            break;
        case '5':
            sky_time_set_mode(SKY_TIME_MODE_REV_STEP);
            break;
        case '6':
            sky_time_set_mode(SKY_TIME_MODE_FWD_STEP);
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

    // this routine fills in the x arg fields az,el,mag

    // interpolated_val = v0 + ((v1 - v0) / (t1 - t0)) * (t - t0)
    #define INTERPOLATE(t,v0,v1,t0,t1) \
        ((double)(v0) + (((double)(v1) - (double)(v0)) / ((double)(t1) - (double)(t0))) * ((double)(t) - (double)(t0)))

    // this routine is only to be called for OBJTYPE_SOLAR
    if (x->type != OBJTYPE_SOLAR) {
        FATAL("BUG: called for invalid obj type %d\n",x->type);
    }

    // preset returns to NO_VALUE
    x->ra  = NO_VALUE;
    x->dec = NO_VALUE;
    x->mag = NO_VALUE;

    // start the search at idx_info hint; the hint will usually be
    // correct, and we can go directly to the interpolation
    if (t >= info[idx].t && t <= info[idx+1].t) {
        goto interpolate;
    }

    // if time is earlier than the idx_info hint then decrement idx
    while (t < info[idx].t) {
        idx--;
        if (idx < 0) {
            ERROR_INTERVAL(1000000, "time %s too early for solar_sys_object %s\n", gmtime_str(t,str), x->name);
            return -1;
        }
    }

    // if time is greater than the idx_info hint then increment idx
    while (t > info[idx+1].t) {
        idx++;
        if (idx > max_info-2) {
            ERROR_INTERVAL(1000000, "time %s too large for solar_sys_object %s\n", gmtime_str(t,str), x->name);
            return -1;
        }
    }

    // if time is now not in range, that is a bug
    if (t < info[idx].t || t > info[idx+1].t) {
        FATAL("BUG: t=%ld info[%d].t=%ld info[%d].t=%ld\n",
              t, idx, info[idx].t, idx+1, info[idx+1].t);
    }

interpolate:
    // determine ra and dec return values by interpolation;
    // don't interpolating mag, it can have NO_VALUE
    x->ra  = INTERPOLATE(t, info[idx].ra, info[idx+1].ra, info[idx].t, info[idx+1].t); 
    x->dec = INTERPOLATE(t, info[idx].dec, info[idx+1].dec, info[idx].t, info[idx+1].t); 
    x->mag = info[idx].mag;
        
    // save idx hint, to be used on subsequent calls
    ssinfo->idx_info = idx;

    // return success
    return 0;
}

void reset(bool all_sky) 
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
    tracking = false;
    sky_view_scale_tbl_idx = 0;
    sky_time_set_mode(SKY_TIME_MODE_CURRENT);
    sky_time_set_step_mode(SKY_TIME_STEP_MODE_DELTA_T,0);
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

void hr2hms(double hr, int * hour, int * minute, double * seconds)
{
    double secs = hr * 3600;

    *hour = secs / 3600;
    secs -= 3600 * *hour;
    *minute = secs / 60;
    secs -= *minute * 60;
    *seconds = secs;
}

char * hr_str(double hr, char *str)
{
    int hour, minute;
    double seconds;

    hr2hms(hr, &hour, &minute, &seconds);
    sprintf(str, "%2.2d:%2.2d:%05.2f", hour, minute, seconds);
    return str;
}

void get_next_day(int * year, int * month, int * day)
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

void get_prior_day(int * year, int * month, int * day)
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

// -----------------  SKY TIME  -------------------------------------------

time_t __sky_time                    = 0;
long   __sky_time_mode               = SKY_TIME_MODE_CURRENT;
int    __sky_time_step_mode          = SKY_TIME_STEP_MODE_DELTA_T;
double __sky_time_step_mode_hr_param = 0;

void sky_time_set_mode(int mode)
{
    __sky_time_mode = mode;
}

void sky_time_get_mode(int * mode)
{
    *mode = __sky_time_mode;
}

void sky_time_set_step_mode(int step_mode, double step_mode_hr_param)
{
    __sky_time_step_mode = step_mode;
    __sky_time_step_mode_hr_param = step_mode_hr_param;
}

void sky_time_get_step_mode(int *step_mode, double *step_mode_hr_param)
{
    *step_mode = __sky_time_step_mode;
    *step_mode_hr_param = __sky_time_step_mode_hr_param;
}

time_t sky_time_get_time(void) 
{
    time_t trise, tset;
    unsigned long time_now;

    static unsigned long time_of_last_call;

    // if this is an early call (less than 100 ms from last call) then
    // return the same sky_time as before; the reason being that when 
    // the display is being panned the display updates more frequently
    // than the normal 100 ms interval
    time_now = microsec_timer();
    if (time_now - time_of_last_call < 90000) {
        return __sky_time;
    }
    time_of_last_call = time_now;

    // determine return time based on time mode
    switch (__sky_time_mode) {
    case SKY_TIME_MODE_CURRENT:
        __sky_time = time(NULL);
        break;
    case SKY_TIME_MODE_PAUSED:
        // no change to __sky_time.
        break;
    case SKY_TIME_MODE_FWD:
    case SKY_TIME_MODE_FWD_STEP:
        switch (__sky_time_step_mode) {
        case SKY_TIME_STEP_MODE_DELTA_T:
            __sky_time = __sky_time + DELTA_T;
            break;
        case SKY_TIME_STEP_MODE_SUNRISE:
            sunrise_sunset(jdconv2(__sky_time)+1, &trise, &tset);
            __sky_time = trise + __sky_time_step_mode_hr_param * 3600;
            break;
        case SKY_TIME_STEP_MODE_SUNSET:
            sunrise_sunset(jdconv2(__sky_time)+1, &trise, &tset);
            __sky_time = tset + __sky_time_step_mode_hr_param * 3600;
            break;
        case SKY_TIME_STEP_MODE_SIDDAY:
            __sky_time = __sky_time + SID_DAY_SECS;
            break;
        case SKY_TIME_STEP_MODE_TIMEOFDAY:
            __sky_time = sky_time_tod_next(__sky_time, __sky_time_step_mode_hr_param);
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
            __sky_time = __sky_time - DELTA_T;
            break;
        case SKY_TIME_STEP_MODE_SUNRISE:
            sunrise_sunset(jdconv2(__sky_time)-1, &trise, &tset);
            __sky_time = trise + __sky_time_step_mode_hr_param * 3600;
            break;
        case SKY_TIME_STEP_MODE_SUNSET:
            sunrise_sunset(jdconv2(__sky_time)-1, &trise, &tset);
            __sky_time = tset + __sky_time_step_mode_hr_param * 3600;
            break;
        case SKY_TIME_STEP_MODE_SIDDAY:
            __sky_time = __sky_time - SID_DAY_SECS;
            break;
        case SKY_TIME_STEP_MODE_TIMEOFDAY:
            __sky_time = sky_time_tod_prior(__sky_time, __sky_time_step_mode_hr_param);
            break;
        default:
            FATAL("BUG: invalid __sky_time_step_mode %d\n", __sky_time_step_mode);
        }
        if (__sky_time_mode == SKY_TIME_MODE_REV_STEP) {
            sky_time_set_mode(SKY_TIME_MODE_PAUSED);
        }
        break;
    default:
        FATAL("invalid sky_time_mode %ld\n", __sky_time_mode);
    }

    // return the time
    return __sky_time;
} 

time_t sky_time_tod_next(time_t t, double hr) 
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

    return timegm(&tm1);
}

time_t sky_time_tod_prior(time_t t, double hr) 
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

    return timegm(&tm1);
}

// -----------------  CONVERT RA/DEC TO AZ/EL UTILS  ----------------------

// INFO
// - https://en.wikipedia.org/wiki/Epoch_(reference_date)#J2000.0
//   The current standard epoch is called "J2000.0" This is defined by 
//   international agreement to be equivalent to:
//    . The Gregorian date January 1, 2000 at approximately 12:00 GMT (Greenwich Mean Time).
//    . The Julian date 2451545.0 TT (Terrestrial Time).[8]
//    . January 1, 2000, 11:59:27.816 TAI (International Atomic Time).[9]
//    . January 1, 2000, 11:58:55.816 UTC (Coordinated Universal Time).[a]
//   NOTE - the above seems to contradicts the result from jdconv() which returns
//          2451545.0 for 1/1/2000 at 12:00:00
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

// based on https://aa.usno.navy.mil/faq/docs/JD_Formula.php
void jd2ymdh(double jd, int *year, int *month, int *day, double *hour)
{
    // 
    // COMPUTES THE GREGORIAN CALENDAR DATE (YEAR,MONTH,DAY)
    // GIVEN THE JULIAN DATE (JD).
    // 

    int JD = jd + .5;
    int I,J,K,L,N;

    L= JD+68569;
    N= 4*L/146097;
    L= L-(146097*N+3)/4;
    I= 4000*(L+1)/1461001;
    L= L-1461*I/4+31;
    J= 80*L/2447;
    K= L-2447*J/80;
    L= J/11;
    J= J+2-12*L;
    I= 100*(N-49)+I+L;

    *year= I;
    *month= J;
    *day = K;

    double jd0 = jdconv(*year, *month, *day, 0);
    *hour = (jd - jd0) * 24;
}

// https://idlastro.gsfc.nasa.gov/ftp/pro/astro/ct2lst.pro
// To convert from Local Civil Time to Local Mean Sidereal Time.
double ct2lst(double lng, double jd)
{
    #define C0 280.46061837
    #define C1 360.98564736629
    #define C2 0.000387933
    #define C3 38710000.0

    double t0, theta, lst;
    double t;

    t0 = jd - JD2000;
    t = t0 / 36525;

    // Compute GST in seconds.
    theta = C0 + (C1 * t0) + t * t * (C2 - t / C3);

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

// -----------------  SUNRISE & SUNSET TIMES  -----------------------------

// https://en.wikipedia.org/wiki/Sunrise_equation#Hour_angle
void sunrise_sunset(double jd, time_t *trise, time_t *tset)
{
    int n;
    double jstar, M, C, lambda, jtransit, declination, hour_angle, jset, jrise;

    int year, month, day, hour, minute;
    double hr, seconds;
    struct tm tm;

    // XXX check constants copied correctly, and double check equations

    // calculate number of julian days since JD2000 epoch
    n = jd - JD2000;

    // mean solar noon
    jstar = n - longitude / 360;

    // solar mean anomaly
    M = (357.5291 + .98560028 * jstar);
    if (M < 0) FATAL("M < 0\n");  // XXX better mod
    while (M > 360) M -= 360; 

    // equation of the center
    C = 1.9148 * SIND(M) + 0.0200 * SIND(2*M) + 0.0003 * SIND(3*M);

    // ecliptic longitude
    lambda = (M +  C + 180 + 102.9372);
    if (lambda < 0) FATAL("lambda < 0\n");  // XXX better mod
    while (lambda > 360) lambda -= 360;

    // solar transit
    jtransit = JD2000 + jstar + 0.0053 * SIND(M) - 0.0069 * SIND(2*lambda);

    // declination of the sun
    declination = ASIND(SIND(lambda) * SIND(23.44));

#if 0
    // hour angle for sun center and no refraction correction
    hour_angle = ACOSD(-TAND(latitude) * TAND(declination));
#else
    // hour angle with correction for refraction and disc diameter
    hour_angle = ACOSD( (SIND(-0.83) - SIND(latitude) * SIND(declination)) /
                       (COSD(latitude) * COSD(declination)));
#endif

    // calculate sunrise and sunset
    jset = jtransit + hour_angle / 360;
    jrise = jtransit - hour_angle / 360;

    // convert jrise to linux time
    if (trise) {
        jd2ymdh(jrise, &year, &month, &day, &hr);
        hr2hms(hr, &hour, &minute, &seconds);
        memset(&tm,0,sizeof(tm));
        tm.tm_sec   = seconds;
        tm.tm_min   = minute;
        tm.tm_hour  = hour;
        tm.tm_mday  = day;
        tm.tm_mon   = month - 1;       // 0 to 11
        tm.tm_year  = year - 1900;   // based 1900
        *trise = timegm(&tm);
    }

    // convert jset to linux time
    if (tset) {
        jd2ymdh(jset, &year, &month, &day, &hr);
        hr2hms(hr, &hour, &minute, &seconds);
        memset(&tm,0,sizeof(tm));
        tm.tm_sec   = seconds;
        tm.tm_min   = minute;
        tm.tm_hour  = hour;
        tm.tm_mday  = day;
        tm.tm_mon   = month - 1;       // 0 to 11
        tm.tm_year  = year - 1900;   // based 1900
        *tset = timegm(&tm);
    }
}

// -----------------  TEST ALGORITHMS  ------------------------------------

void unit_test(void) 
{
    double lat_save, long_save;

    // save latitude and longitude, and replace them with unit test values
    lat_save = latitude;
    long_save = longitude;
    latitude = 42.422986;
    longitude= -71.623798;

    // test jdconv ...
    // https://www.aavso.org/jd-calculator; also
    // refer to https://en.wikipedia.org/wiki/Epoch_(astronomy)#Julian_years_and_J2000
    { double jd;
    jd = jdconv(2000,1,1,12);
    if (jd != 2451545.0) {
        FATAL("jd %f should be 2451545.0\n", jd);
    } }

    // test jd2ymdh
    { int y_act, m_act, d_act; double h_act;
      int y_exp, m_exp, d_exp; double h_exp;
      double jd, devi;

    y_exp = 2000; m_exp = 1; d_exp = 1; h_exp = 0;
    jd = jdconv(y_exp, m_exp, d_exp, h_exp);
    jd2ymdh(jd, &y_act, &m_act, &d_act, &h_act);
    if (y_act != y_exp || m_act != m_exp || d_act != d_exp || !is_close(h_act,h_exp,.000001,&devi)) {
        FATAL("jd2ymdh exp %d %d %d %f actual %d %d %d %f\n",
              y_exp, m_exp, d_exp, h_exp, y_act, m_act, d_act, h_act);
    }

    y_exp = 2010; m_exp = 6; d_exp = 29; h_exp = 15.1234;
    jd = jdconv(y_exp, m_exp, d_exp, h_exp);
    jd2ymdh(jd, &y_act, &m_act, &d_act, &h_act);
    if (y_act != y_exp || m_act != m_exp || d_act != d_exp || !is_close(h_act,h_exp,.000001,&devi)) {
        FATAL("jd2ymdh exp %d %d %d %f actual %d %d %d %f\n",
              y_exp, m_exp, d_exp, h_exp, y_act, m_act, d_act, h_act);
    }

    y_exp = 2018; m_exp = 12; d_exp = 22; h_exp = 15.75;
    jd = jdconv(y_exp, m_exp, d_exp, h_exp);
    jd2ymdh(jd, &y_act, &m_act, &d_act, &h_act);
    if (y_act != y_exp || m_act != m_exp || d_act != d_exp || !is_close(h_act,h_exp,.000001,&devi)) {
        FATAL("jd2ymdh exp %d %d %d %f actual %d %d %d %f\n",
              y_exp, m_exp, d_exp, h_exp, y_act, m_act, d_act, h_act);
    } }

    // test local sidereal time ...
    // from date cmd: Fri Dec  7 20:53:37 EST 2018
    // from https://tycho.usno.navy.mil/cgi-bin/sidereal-post.sh
    //    02:12:44.9 LST         2+12/60+44.9/3600 = 2.21247
    //    Longitude -72 00 00
    { double jd, lst, lst_deviation;
    jd = jdconv(2018,12,8,1.8936);  
    lst = ct2lst(-72, jd);
    if (!is_close(lst, 2.21247, 0.00001, &lst_deviation)) {
        FATAL("lst_deviation = %f, exceeds limit\n", lst_deviation);
    }
    INFO("lst_deviation = %f\n", lst_deviation);
    }

    // this webiste has a radec2azel convert code, which I did not use; 
    // however I did use the sample problem provided in the comments ...
    // https://www.mathworks.com/matlabcentral/fileexchange/26458-convert-right-ascension-and-declination-to-azimuth-and-elevation
    // 
    // Worked Example: http://www.stargazing.net/kepler/altaz.html
    // [Az El] = RaDec2AzEl(344.95,42.71667,52.5,-1.91667,'1997/03/14 19:00:00')
    // [311.92258 22.40100] = RaDec2AzEl(344.95,42.71667,52.5,-1.91667,'1997/03/14 19:00:00')
    { double ra, dec, az, el, lat, lng, jd, lst, az_exp, el_exp, az_deviation, el_deviation;
      int ret;
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
    if (!is_close(az, az_exp, 0.00001, &az_deviation)) {
        FATAL("az_deviation = %f, exceeds limit\n", az_deviation);
    }
    if (!is_close(el, el_exp, 0.00001, &el_deviation)) {
        FATAL("el_deviation = %f, exceeds limit\n", el_deviation);
    }
    INFO("az_deviatin = %f  el_deviation = %f\n", az_deviation, el_deviation);
    }

    // print list of solar_sys objects and their ra,dec,mag,az,el
    { time_t t;
      double lst, az, el;
      int i, ret;
    t = time(NULL);
    lst = ct2lst(longitude, jdconv2(t));
    INFO("            NAME         RA        DEC        MAG         AZ         EL\n");
    for (i = 0; i < max_obj; i++) {
        obj_t *x = &obj[i];

        if (x->type != OBJTYPE_SOLAR) {
            continue;
        }

        compute_ss_obj_ra_dec_mag(x, t);
        ret = radec2azel(&az, &el, x->ra, x->dec, lst, latitude);
        if (ret != 0) {
            FATAL("radec2azel failed\n");
        }

        INFO("%16s %10.4f %10.4f %10.1f %10.4f %10.4f\n", x->name, x->ra, x->dec, x->mag, az, el);
    } }

    // time how long to convert all stellar objects to az/el
    { double lst, az, el;
      time_t t;
      int i, ret;
      unsigned long start_us, duration_us;
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
    }

    // test sunrise and sunset
    // https://www.timeanddate.com/sun/usa/marlborough?month=3&year=2018
    { time_t trise, tset;
      struct tm *tm;
      double jd;

    jd = jdconv(2018, 3, 13, 12.01);
    sunrise_sunset(jd, &trise, &tset);

    tm = localtime(&tset);
    if (tm->tm_year   != 2018-1900 ||
        tm->tm_mon    != 3-1 ||
        tm->tm_mday   != 13 ||
        tm->tm_hour   != 18 ||
        tm->tm_min    != 50) 
    {
        FATAL("sunset %s\n", asctime(tm));
    }
    
    tm = localtime(&trise);
    if (tm->tm_year   != 2018-1900 ||
        tm->tm_mon    != 3-1 ||
        tm->tm_mday   != 13 ||
        tm->tm_hour   != 7 ||
        tm->tm_min    != 2) 
    {
        FATAL("sunset %s\n", asctime(tm));
    } }

    // restore latite/longitude
    latitude = lat_save;
    longitude = long_save;

    INFO("tests passed\n");
}

bool is_close(double act, double exp, double allowed_deviation, double * deviation)
{
    bool is_close;

    if (act == exp) {
        *deviation = 0;
        return true;
    }

    if (exp != 0) {
        *deviation = fabs((act - exp) / exp);
    } else {
        *deviation = fabs((act - exp) / act);
    }

    is_close = (*deviation < allowed_deviation);

    return is_close;
}
