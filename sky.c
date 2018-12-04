#include "common.h"

//
// defines
//

#define MAX_STELLAR_OBJECT      200000
#define MAX_SOLAR_SYS_OBJECT    20 
#define MAX_NAME                32
#define MAX_INFO_TBL            100000

//
// typedefs
//

typedef struct {
    char  name[MAX_NAME];
    float ra;
    float dec;
    float mag;
} stellar_object_t;

typedef struct {
    char name[MAX_NAME];
    int max_info;
    int idx_info;
    struct info_s {
        time_t t;
        float ra;
        float dec;
        float mag;
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
int get_solar_object_values(solar_sys_object_t *x, time_t t, float *ra, float *dec, float *mag);
char * gmtime_str(time_t t);

// -----------------  SKY INIT  -------------------------------------------

int sky_init(void) 
{
    int ret;

    INFO("sizeof(stellar_object)   = %ld MB\n", sizeof(stellar_object)/MB);
    INFO("sizeof(solar_sys_object) = %ld MB\n", sizeof(solar_sys_object)/MB);
    INFO("TIME NOW                 = %s UTC\n", gmtime_str(time(NULL)));

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
    float ra, dec, mag;

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

        if (sscanf(ra_str, "%f", &ra) != 1) {
            ERROR("filename=%s line=%d invalid ra='%s'\n", filename, line, ra_str);
            return -1;
        }
        if (sscanf(dec_str, "%f", &dec) != 1) {
            ERROR("filename=%s line=%d invalid dec='%s'\n", filename, line, dec_str);
            return -1;
        }
        if (sscanf(mag_str, "%f", &mag) != 1) {
            ERROR("filename=%s line=%d invalid mag='%s'\n", filename, line, mag_str);
            return -1;
        }

        strncpy(stellar_object[max_stellar_object].name, proper_str, MAX_NAME);
        stellar_object[max_stellar_object].ra  = ra;
        stellar_object[max_stellar_object].dec = dec;
        stellar_object[max_stellar_object].mag = mag;
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
    float ra, dec, mag;
    time_t t;
    struct tm tm;

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
        tm.tm_min   = minute;
        tm.tm_hour  = hour;
        tm.tm_mday  = day;
        tm.tm_mon   = month;       // 0 to 11
        tm.tm_year  = year-1900;   // based 1900
        t = timegm(&tm);
        }

        // convert the righ-ascension, declination, and magniture strings to floating point
        if (sscanf(ra_str, "%f", &ra) != 1) {
            ERROR("filename=%s line=%d invalid ra='%s'\n", filename, line, ra_str);
            return -1;
        }
        if (sscanf(dec_str, "%f", &dec) != 1) {
            ERROR("filename=%s line=%d invalid dec='%s'\n", filename, line, dec_str);
            return -1;
        }
        if (sscanf(mag_str, "%f", &mag) != 1) {
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
    //      xxxxxxxxxxxxxxxx xxxxxxxxxx xxxxxxxxxx xxxxxxxxxx
    printf("            NAME         RA        DEC        MAG\n");
    for (i = 0; i < max_solar_sys_object; i++) {
        solar_sys_object_t * x = &solar_sys_object[i];
        float ra, dec, mag;
        int ret;
        time_t t = time(NULL);
        ret = get_solar_object_values(x, t, &ra, &dec, &mag);
        if (ret != 0) {
            ERROR("get_solar_object_values failed\n");
            return -1;
        }
        printf("%16s %10.4f %10.4f %10.1f\n", x->name, ra, dec, mag);
    }

    // success
    return 0;
}

// -----------------  SKY UTILS -------------------------------------------

int get_solar_object_values(solar_sys_object_t *x, time_t t, float *ra, float *dec, float *mag)
{
    struct info_s *info     = x->info;
    int            max_info = x->max_info;
    int            idx      = x->idx_info;

    // interpolated_val = v0 + ((v1 - v0) / (t1 - t0)) * (t - t0)
    #define INTERPOLATE(t,v0,v1,t0,t1) \
        ((float)(v0) + (((float)(v1) - (float)(v0)) / ((float)(t1) - (float)(t0))) * ((float)(t) - (float)(t0)))

    if (t >= info[idx].t && t <= info[idx+1].t) {
        goto interpolate;
    }

    while (t < info[idx].t) {
        idx--;
        if (idx < 0) {
            ERROR("time %s too early for solar_sys_object %s\n", gmtime_str(t), x->name);
            return -1;
        }
    }

    while (t > info[idx+1].t) {
        idx++;
        if (idx > max_info-2) {
            ERROR("time %s too large for solar_sys_object %s\n", gmtime_str(t), x->name);
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

#if 0
           struct tm {
               int tm_sec;    /* Seconds (0-60) */
               int tm_min;    /* Minutes (0-59) */
               int tm_hour;   /* Hours (0-23) */
               int tm_mday;   /* Day of the month (1-31) */
               int tm_mon;    /* Month (0-11) */
               int tm_year;   /* Year - 1900 */
               int tm_wday;   /* Day of the week (0-6, Sunday = 0) */
               int tm_yday;   /* Day in the year (0-365, 1 Jan = 0) */
               int tm_isdst;  /* Daylight saving time */
           };
#endif
char *gmtime_str(time_t t)
{
    struct tm *tm;
    static char str[100];

    tm = gmtime(&t);
    // example format: 2018-Dec-01 00:00
    sprintf(str, "%4d-%s-%2.2d %2.2d:%2.2d",
            tm->tm_year+1900, 
            month_tbl[tm->tm_mon],
            tm->tm_mday,
            tm->tm_hour,
            tm->tm_min);
    return str;
}

// -----------------  SKY PANE HANDLER  -----------------------------------

int sky_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event) 
{
    struct {
        int tbd;
    } * vars = pane_cx->vars;
    rect_t * pane = &pane_cx->pane;

    #define SDL_EVENT_MOUSE_MOTION   (SDL_EVENT_USER_DEFINED + 0)
    #define SDL_EVENT_MOUSE_WHEEL    (SDL_EVENT_USER_DEFINED + 1)

    // ----------------------------
    // -------- INITIALIZE --------
    // ----------------------------

    if (request == PANE_HANDLER_REQ_INITIALIZE) {
        vars = pane_cx->vars = calloc(1,sizeof(*vars));
        return PANE_HANDLER_RET_NO_ACTION;
    }

    // ------------------------
    // -------- RENDER --------
    // ------------------------

    if (request == PANE_HANDLER_REQ_RENDER) {
        rect_t * pane = &pane_cx->pane;
        return PANE_HANDLER_RET_NO_ACTION;
    }

    // -----------------------
    // -------- EVENT --------
    // -----------------------

    if (request == PANE_HANDLER_REQ_EVENT) {
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


