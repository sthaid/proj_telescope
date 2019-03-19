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

// XXX problems ...
// 1) I am concerned that the .0008 JD or 68 second difference between UTC
//     and TT is not being accounted for in the code. However, when the .0008
//     is added to this program cross checks with other websites that provide
//     sunrise/sunset time, and the position of the moon, had greater 
//     discrepency. References:
//     - https://en.wikipedia.org/wiki/Epoch_(reference_date)#J2000.0
//         info on JD2000
//     - https://aa.usno.navy.mil/data/docs/AltAz.php
//         az/el of the moon or sun
//     - https://www.timeanddate.com/sun/usa/marlborough?month=3&year=2018
//         sunrise/sunset times
// 2) Difference between the moon position provided by 
//    https://aa.usno.navy.mil/data/docs/AltAz.php and this program of
//    approximately 0.2 degrees in azimuth and elevation.
//    This difference is without the 0.0008 included in the code. When the
//    0.0008 is included the difference is larger.

// XXX try cross checking azel for stars using other web sites

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stddef.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <limits.h>
#include <assert.h>
#include <math.h>

#include "util_misc.h"
#include "util_sky.h"

//
// defines
//

#define MAX_OBJ      200000
#define MAX_OBJ_NAME 32

#define SIND(x)   (sin((x)*DEG2RAD))
#define COSD(x)   (cos((x)*DEG2RAD))
#define TAND(x)   (tan((x)*DEG2RAD))
#define ACOSD(x)  (acos(x)*RAD2DEG)
#define ASIND(x)  (asin(x)*RAD2DEG)

#define RAD2DEG (180. / M_PI)
#define DEG2RAD (M_PI / 180.)
#define HR2RAD  (M_PI / 12.)

#define JD2000 2451545.0

#define SIZEOF_SOLAR_SYS_OBJ_INFO_T(n) (sizeof(solar_sys_obj_info_t) + sizeof(struct info_s) * (n))

#define INFO2(fmt, args...) \
    do { \
        if (enable_info_prints) { \
            INFO(fmt, ## args); \
        } \
    } while (0)

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
    double az;
    double el;
    time_t t;
    solar_sys_obj_info_t * ssinfo;
} obj_t;

//
// variables
//

static obj_t obj[MAX_OBJ];

static char *month_tbl[12] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };

static bool  incl_filtering_enabled;
static char *incl_obj[1000];
static bool  incl_all_stars;
static bool  incl_all_named_stars;
static bool  incl_all_planets;
static bool  incl_all_placemarks;

static bool enable_info_prints;

//
// prototypes
//

static int read_stellar_data(char * filename);
static int read_solar_sys_data(char * filename);
static int read_place_marks(char * filename);
static int obj_sanity_checks(void);
static int compute_ss_obj_ra_dec_mag(obj_t *x, time_t t);
static void util_sky_unit_test(void);
static bool is_close(double act, double exp, double allowed_deviation, double * deviation);

// -----------------  UTIL SKY INIT  --------------------------------------

// the main purpose of these routines is to read the sky_data files

int util_sky_init(char *incl_obj_str, bool run_unit_test, bool enable_info_prints_arg)
{
    int ret;
    char str[100];
    bool first = true;
    double lst = ct2lst(longitude, jdconv2(time(NULL)));

    enable_info_prints = enable_info_prints_arg;

    INFO2("UTC now    = %s\n", gmtime_str(time(NULL),str));
    INFO2("LCL now    = %s\n", localtime_str(time(NULL),str));
    INFO2("Latitude   = %12s  % .6f\n", hr_str(latitude,str), latitude);
    INFO2("Longitude  = %12s  % .6f\n", hr_str(longitude,str), longitude);
    INFO2("LST        = %s\n", hr_str(lst,str));
    INFO2("sizeof obj = %ld MB\n", sizeof(obj)/0x100000);

    // parse incl_obj_str, which is a comma seperated list of 
    // objects to be included; if the list is not supplied
    // then all objects will be included; special case strings:
    // - "stars"       - include all stars
    // - "named_stars" - include all named stars
    // - "planets"     - include all planets
    // - "placemarks"  - include all placemarks
    if (incl_obj_str) {
        int max_incl_obj=0;
        incl_filtering_enabled = true;
        while (true) {
            char *name = strtok(first ? incl_obj_str : NULL, ",");
            first = false;
            if (name == NULL) {
                break;
            }
            if (strcasecmp(name, "stars") == 0) {
                incl_all_stars = true;
            } else if (strcasecmp(name, "named_stars") == 0) {
                incl_all_named_stars = true;
            } else if (strcasecmp(name, "planets") == 0) {
                incl_all_planets = true;
            } else if (strcasecmp(name, "placemarks") == 0) {
                incl_all_placemarks = true;
            } else {
                incl_obj[max_incl_obj++] = name;
            }
        }
    }

    // read positions of stars from hygdata_v3.csv"
    ret = read_stellar_data("sky_data/hygdata_v3.csv");
    if (ret < 0) {
        return ret;
    }

    // read positions of solar sys objects (planets, moons, etc) from solar_sys_data.csv
    ret = read_solar_sys_data("sky_data/solar_sys_data.csv");
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
    INFO2("max_object  = %d\n", max_obj);

    // run some unit tests (optional)
    if (run_unit_test) {
        util_sky_unit_test();
    }

    // success
    return 0;
}

static int read_stellar_data(char * filename)
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
    int line=0, num_added=0, i;
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
    bool include;

    // open file
    fp = fopen(filename, "r");
    if (fp == NULL) {
        ERROR("failed to open %s\n", filename);
        return -1;
    }

    // read and parse all lines
    while (fgets(str, sizeof(str), fp) != NULL) {
        line++;

        // check if obj table is full
        if (max_obj >= MAX_OBJ) {
            ERROR("filename=%s line=%d obj table is full\n", filename, line);
            return -1;
        }

        // parse s into the various csv fields
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

        // spot check csv file header line
        if (line == 1) {
            if (strcmp(proper_str, "proper") || strcmp(ra_str, "ra") || strcmp(dec_str, "dec") || strcmp(mag_str, "mag")) {
                ERROR("csv file header line incorrect, proper='%s' ra=%s dec=%s mag=%s\n", proper_str, ra_str, dec_str, mag_str);
                return -1;
            }
            continue;
        } 

        // don't know why hygdata_v3.csv has Sol entry at ra=0 dec=0;
        // so just ignore it
        if (strcmp(proper_str, "Sol") == 0) {
            continue;
        }

        // determine if this object is to be included;
        // if not then continue
        if (incl_filtering_enabled) {
            if (incl_all_stars) {
                include = true;
            } else if (incl_all_named_stars && proper_str[0] != '\0') {
                include = true;
            } else {
                include = false;
                for (i = 0; incl_obj[i]; i++) {
                    if (strcasecmp(proper_str, incl_obj[i]) == 0) {
                        include = true;
                        break;
                    }
                }
            }
        } else {
            include = true;
        }
        if (!include) {
            continue;
        }

        // scan the strings to get ra,dec, and mag
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

        // add this entry to obj table
        strncpy(obj[max_obj].name, proper_str, MAX_OBJ_NAME-1);
        obj[max_obj].type   = OBJTYPE_STELLAR;
        obj[max_obj].ra     = ra * 15.;
        obj[max_obj].dec    = dec;
        obj[max_obj].mag    = mag;
        obj[max_obj].ssinfo = NULL;
        DEBUG("%16s %10.4f %10.4f %10.4f\n", proper_str, ra, dec, mag);
        max_obj++;
        num_added++;
    }

    // close file
    fclose(fp);

    // success
    INFO2("added %d stellar_objects from %s\n", num_added, filename);
    return 0;
}

static int read_solar_sys_data(char *filename)
{
    // format, example:
    //   # Venus
    //    2018-Dec-01 00:00, , ,207.30643, -9.79665,  -4.87,  1.44,

    FILE *fp;
    int line=0, num_added=0, len, i;
    obj_t *x = NULL;
    solar_sys_obj_info_t *ssinfo = NULL;
    char str[10000], *s, *name;
    char *date_str, *ra_str, *dec_str, *mag_str;
    char *not_used_str __attribute__ ((unused));
    double ra, dec, mag;
    time_t t;
    bool include=true;

    // open
    fp = fopen(filename, "r");
    if (fp == NULL) {
        ERROR("failed to open %s\n", filename);
        return -1;
    }

    // read and parse all lines
    while (fgets(str, sizeof(str), fp) != NULL) {
        line++;
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

            // determine if this object is to be included;
            // if not then continue
            if (incl_filtering_enabled) {
                if (incl_all_planets) {
                    include = true;
                } else {
                    include = false;
                    for (i = 0; incl_obj[i]; i++) {
                        if (strcasecmp(name, incl_obj[i]) == 0) {
                            include = true;
                            break;
                        }
                    }
                }
            } else {
                include = true;
            }
            if (!include) {
                continue;
            }

            // alloc ssinfo, it will be realloced in increments of 10000 struct info_s as needed
            ssinfo = malloc(SIZEOF_SOLAR_SYS_OBJ_INFO_T(0));
            ssinfo->max_info = 0;
            ssinfo->idx_info = 0;

            // init obj fields
            x = &obj[max_obj];
            strncpy(x->name, name, MAX_OBJ_NAME-1);
            x->type = OBJTYPE_SOLAR;
            x->ra       = NO_VALUE;
            x->dec      = NO_VALUE;
            x->mag      = NO_VALUE;
            x->ssinfo   = ssinfo;

            // update counters
            max_obj++;
            num_added++;
            continue;
        }

        // if not including this object then continue
        if (!include) {
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
    }

    // close
    fclose(fp);

    // success
    INFO2("added %d solar_sys_objects from %s\n", num_added, filename);
    return 0;
}

static int read_place_marks(char *filename)
{
    FILE * fp;
    int line=0, num_added=0, cnt, i;
    char str[1000], name[100];
    double ra, dec;
    bool include;

    // open
    fp = fopen(filename, "r");
    if (fp == NULL) {
        ERROR("failed to open %s\n", filename);
        return -1;
    }

    // read and parse all lines
    while (fgets(str, sizeof(str), fp) != NULL) {
        line++;

        // skip comment lines
        if (str[0] == '#') {
            continue;
        }

        // scan the line for placemark name, ra, and dec
        cnt = sscanf(str, "%s %lf %lf\n", name, &ra, &dec);
        if (cnt != 3) {
            ERROR("filename=%s line=%d invalid, scan cnt %d\n", filename, line, cnt);
            return -1;
        }
        DEBUG("PLACE_MARK '%s' %f %f\n", name, ra, dec);

        // determine if this object is to be included;
        // if not then continue
        if (incl_filtering_enabled) {
            if (incl_all_placemarks) {
                include = true;
            } else {
                include = false;
                for (i = 0; incl_obj[i]; i++) {
                    if (strcasecmp(name, incl_obj[i]) == 0) {
                        include = true;
                        break;
                    }
                }
            }
        } else {
            include = true;
        }
        if (!include) {
            continue;
        }

        // add the placemark to the obj table
        // XXX fix this stringop-truncation some better way
#if 0
        strncpy(obj[max_obj].name, name, MAX_OBJ_NAME-1);
#else
        memcpy(obj[max_obj].name, name, MAX_OBJ_NAME-1);
#endif
        obj[max_obj].type   = OBJTYPE_PLACE_MARK;;
        obj[max_obj].ra     = ra;
        obj[max_obj].dec    = dec;
        obj[max_obj].mag    = NO_VALUE;
        obj[max_obj].ssinfo = NULL;
        max_obj++;
        num_added++;
    }

    // close
    fclose(fp);

    // success
    INFO2("added %d place_mark_objects from %s\n", num_added, filename);
    return 0;
}

static int obj_sanity_checks(void)
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

// -----------------  GET_OBJ  --------------------------------------------

int get_obj(int i, time_t t, double lst, char **name, int *type, double *ra, double *dec, double *mag, double *az, double *el)
{
    obj_t *x = &obj[i];

    // sanity checks
    if (i < 0 || i >= max_obj) {
        FATAL("BUG: obj %d out of range, max_obj=%d\n", i, max_obj);
    }
    if (x->type == OBJTYPE_NONE) {
        FATAL("BUG: obj %d is OBJTYPE_NONE\n", i);
    }
    if (t == 0) {
        FATAL("BUG: time 0 not allowed\n");
    }

    // if called with the same time as a prior call then 
    // return prior values
    if (t == x->t) {
        goto done;
    }

    // if processing solar-sys object then compute its current ra, dec, and mag
    if (x->type == OBJTYPE_SOLAR) {
        if (compute_ss_obj_ra_dec_mag(x, t) != 0) {
            // XXX limit number of prints, or don't print more than once per 10 secs
            ERROR("time out of range for solar-sys-obj %s\n", x->name);
            return -1;   // ERROR_INTERVAL ?
        }
    }

    // get az/el from ra/dec
    radec2azel(&x->az, &x->el, x->ra, x->dec, lst, latitude);

    // save time called
    x->t = t;

done:
    // return values
    if (name) *name = x->name;
    if (type) *type = x->type;
    if (ra)   *ra   = x->ra;
    if (dec)  *dec  = x->dec;
    if (mag)  *mag  = x->mag;
    if (az)   *az   = x->az;
    if (el)   *el   = x->el;
    return 0;
}

char * get_obj_name(int i) 
{
    // sanity checks
    if (i < 0 || i >= max_obj) {
        FATAL("BUG: obj %d out of range, max_obj=%d\n", i, max_obj);
    }
    if (obj[i].type == OBJTYPE_NONE) {
        FATAL("BUG: obj %d is OBJTYPE_NONE\n", i);
    }

    // return the name
    return obj[i].name;
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
    *az = raz * RAD2DEG + 180.;
    *el = rel * RAD2DEG;
    if (*az >= 360) *az -= 360;

    // sanity check result
    if (*el < -90 || *el > 90 || *az < 0 || *az >= 360) {
        WARN("ra=%f dec=%f cvt to az=%f el=%f, result out of range\n",
             ra, dec, *az, *el);
    }
}

// http://cosmology.berkeley.edu/group/cmbanalysis/forecast/idl/azel2radec.pro
// To convert from horizon coordinates (azimuth-elevation)
// to celestial coordinates (right ascension-declination)
void azel2radec(double *ra, double *dec, double az, double el, double lst, double lat)
{
    double rlat, raz, rel, rdec, rha;

    // converting from degrees to radians
    // note: I added the '- 180'
    rlat = lat * DEG2RAD;
    raz = (az - 180) * DEG2RAD;
    rel = el * DEG2RAD;

    // working out declination
    rdec = asin( sin(rel)*sin(rlat)-cos(rel)*cos(raz)*cos(rlat) );
    *dec = rdec * RAD2DEG;

    // working out right ascension
    rha = atan2(cos(rel)*sin(raz), sin(rel)*cos(rlat)+cos(rel)*cos(raz)*sin(rlat));
    *ra = ((lst * HR2RAD) - rha) * RAD2DEG;

    // bring ra into range 0 to 260
    while (*ra >= 360) *ra -= 360;
    while (*ra < 0) *ra += 360;

    // if dec is +/- 90 then return ra=0
    if (fabs(fabs(*dec)-90) < .000001) {
        *ra = 0;
    }

    // sanity check
    if (*ra < 0 || *ra > 360 || *dec < -90 || *dec > 90) {
        WARN("az=%f el=%f cvt to ra=%f dec=%f, result of of range\n",
             az, el, *ra, *dec);
    }
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

    // calculate number of julian days since JD2000 epoch
    // XXX The actual equation from wikipedia website includes
    //     '0.0008 is the fractional Julian Day for leap seconds and terrestrial time.'
    //     Including the .0008 the equatin is 'n = jd - JD2000 + 0.0008'
    n = jd - JD2000;

    // mean solar noon
    jstar = n - longitude / 360;

    // solar mean anomaly
    M = (357.5291 + .98560028 * jstar);
    if (M < 0) FATAL("BUG: M < 0\n");
    while (M > 360) M -= 360; 

    // equation of the center
    C = 1.9148 * SIND(M) + 0.0200 * SIND(2*M) + 0.0003 * SIND(3*M);

    // ecliptic longitude
    lambda = (M +  C + 180 + 102.9372);
    if (lambda < 0) FATAL("BUG: lambda < 0\n");
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

// -----------------  MISC  -----------------------------------------------

static int compute_ss_obj_ra_dec_mag(obj_t *x, time_t t)
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

void hr2hms(double hr, int * hour, int * minute, double * seconds)
{
    double secs = hr * 3600;

    *hour = secs / 3600;
    secs -= 3600 * *hour;
    *minute = secs / 60;
    secs -= *minute * 60;
    *seconds = secs;
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

// can be used for hour, latitude, or longitude
char * hr_str(double hr, char *str)
{
    int hour, minute;
    double seconds;
    char *sign_str = "";

    if (hr < 0) {
        hr = - hr;
        sign_str = "-";
    }

    hr2hms(hr, &hour, &minute, &seconds);
    sprintf(str, "%s%d:%2.2d:%05.2f", sign_str, hour, minute, seconds);
    return str;
}

// -----------------  TEST ALGORITHMS  ------------------------------------

static void util_sky_unit_test(void) 
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
        FATAL("UNIT_TEST_FAILED: jd %f should be 2451545.0\n", jd);
    } }

    // test jd2ymdh
    { int y_act, m_act, d_act; double h_act;
      int y_exp, m_exp, d_exp; double h_exp;
      double jd, devi;

    y_exp = 2000; m_exp = 1; d_exp = 1; h_exp = 0;
    jd = jdconv(y_exp, m_exp, d_exp, h_exp);
    jd2ymdh(jd, &y_act, &m_act, &d_act, &h_act);
    if (y_act != y_exp || m_act != m_exp || d_act != d_exp || !is_close(h_act,h_exp,.000001,&devi)) {
        FATAL("UNIT_TEST_FAILED: jd2ymdh exp %d %d %d %f actual %d %d %d %f\n",
              y_exp, m_exp, d_exp, h_exp, y_act, m_act, d_act, h_act);
    }

    y_exp = 2010; m_exp = 6; d_exp = 29; h_exp = 15.1234;
    jd = jdconv(y_exp, m_exp, d_exp, h_exp);
    jd2ymdh(jd, &y_act, &m_act, &d_act, &h_act);
    if (y_act != y_exp || m_act != m_exp || d_act != d_exp || !is_close(h_act,h_exp,.000001,&devi)) {
        FATAL("UNIT_TEST_FAILED: jd2ymdh exp %d %d %d %f actual %d %d %d %f\n",
              y_exp, m_exp, d_exp, h_exp, y_act, m_act, d_act, h_act);
    }

    y_exp = 2018; m_exp = 12; d_exp = 22; h_exp = 15.75;
    jd = jdconv(y_exp, m_exp, d_exp, h_exp);
    jd2ymdh(jd, &y_act, &m_act, &d_act, &h_act);
    if (y_act != y_exp || m_act != m_exp || d_act != d_exp || !is_close(h_act,h_exp,.000001,&devi)) {
        FATAL("UNIT_TEST_FAILED: jd2ymdh exp %d %d %d %f actual %d %d %d %f\n",
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
        FATAL("UNIT_TEST_FAILED: lst_deviation = %f, exceeds limit\n", lst_deviation);
    }
    INFO2("lst_deviation = %f\n", lst_deviation);
    }

    // this webiste has a radec2azel convert code, which I did not use; 
    // however I did use the sample problem provided in the comments ...
    // https://www.mathworks.com/matlabcentral/fileexchange/26458-convert-right-ascension-and-declination-to-azimuth-and-elevation
    // 
    // Worked Example: http://www.stargazing.net/kepler/altaz.html
    // [Az El] = RaDec2AzEl(344.95,42.71667,52.5,-1.91667,'1997/03/14 19:00:00')
    // [311.92258 22.40100] = RaDec2AzEl(344.95,42.71667,52.5,-1.91667,'1997/03/14 19:00:00')
    { double ra, dec, az, el, lat, lng, jd, lst, az_exp, el_exp, az_deviation, el_deviation;
    ra  = 344.95;
    dec = 42.71667;
    lat = 52.5;
    lng = -1.91667;
    jd = jdconv(1997, 3, 14, 19);
    lst = ct2lst(lng, jd);
    az_exp = 311.92258;
    el_exp = 22.40100;
    radec2azel(&az, &el, ra, dec, lst, lat);
    DEBUG("calc: az=%f el=%f  expected: az=%f el=%f\n", az, el, az_exp, el_exp);
    if (!is_close(az, az_exp, 0.00001, &az_deviation)) {
        FATAL("UNIT_TEST_FAILED: az_deviation = %f, exceeds limit\n", az_deviation);
    }
    if (!is_close(el, el_exp, 0.00001, &el_deviation)) {
        FATAL("UNIT_TEST_FAILED: el_deviation = %f, exceeds limit\n", el_deviation);
    }
    INFO2("az_deviation = %f  el_deviation = %f\n", az_deviation, el_deviation);
    }

    // print list of solar_sys objects and their ra,dec,mag,az,el
    { time_t t;
      double lst, az, el;
      int i;
#if 1
    t = time(NULL);
#else
    struct tm tm;
    memset(&tm,0,sizeof(tm));
    tm.tm_year  = 119;         // based 1900
    tm.tm_mon   = 0;           // 0 to 11
    tm.tm_mday  = 23;  
    tm.tm_hour  = 12;  
    tm.tm_min   = 0;
    tm.tm_sec   = 0;
    t = timelocal(&tm);
#endif
    lst = ct2lst(longitude, jdconv2(t));
    INFO2("            NAME         RA        DEC        MAG         AZ         EL\n");
    for (i = 0; i < max_obj; i++) {
        obj_t *x = &obj[i];

        if (x->type != OBJTYPE_SOLAR) {
            continue;
        }

        compute_ss_obj_ra_dec_mag(x, t);
        radec2azel(&az, &el, x->ra, x->dec, lst, latitude);

        INFO2("%16s %10.4f %10.4f %10.1f %10.4f %10.4f\n", x->name, x->ra, x->dec, x->mag, az, el);
    } }

    // time how long to convert all stellar objects to az/el
    { double lst, az, el;
      time_t t;
      int i;
      uint64_t start_us, duration_us;
    start_us = microsec_timer();
    t = time(NULL);
    lst = ct2lst(longitude, jdconv2(t));
    for (i = 0; i < max_obj; i++) {
        obj_t * x = &obj[i];
        if (obj->type == OBJTYPE_SOLAR) {
            compute_ss_obj_ra_dec_mag(x, t);
        }
        radec2azel(&az, &el, x->ra, x->dec, lst, latitude);
    }
    duration_us = microsec_timer() - start_us;
    INFO2("radec2azel perf: %d objects in %ld ms\n", max_obj, duration_us/1000);
    }

    // test azel2radec for all objects at the current time; do this by first
    // converting the object's ra/dec to az/el and then use azel2radec to convert back
    { time_t t;
      int i;
      double lst, az, el, ra, dec;
      double ra_deviation, dec_deviation;
      bool ra_is_close, dec_is_close;
    t = time(NULL);
    lst = ct2lst(longitude, jdconv2(t));
    for (i = 0; i < max_obj; i++) {
        obj_t * x = &obj[i];
        if (obj->type == OBJTYPE_SOLAR) {
            compute_ss_obj_ra_dec_mag(x, t);
        }
        radec2azel(&az, &el, x->ra, x->dec, lst, latitude);
        azel2radec(&ra, &dec, az, el, lst, latitude);

        ra_is_close = is_close(ra, x->ra, .0000001, &ra_deviation);
        dec_is_close = is_close(dec, x->dec, .0000001, &dec_deviation);
        if (!ra_is_close || !dec_is_close) {
            FATAL("UNIT_TEST_FAILED: objname %s: radec %f %f -> azel %f %f -> radec %f %f (deviation %f %f)\n",
                  x->name, x->ra, x->dec, az, el, ra, dec,
                  ra_deviation, dec_deviation);
        }
    } }

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
        FATAL("UNIT_TEST_FAILED: sunset %s\n", asctime(tm));
    }
    
    tm = localtime(&trise);
    if (tm->tm_year   != 2018-1900 ||
        tm->tm_mon    != 3-1 ||
        tm->tm_mday   != 13 ||
        tm->tm_hour   != 7 ||
        tm->tm_min    != 2) 
    {
        FATAL("UNIT_TEST_FAILED: sunset %s\n", asctime(tm));
    } }

    // manual test: the position of the moon ...
    // 1) https://aa.usno.navy.mil/data/docs/AltAz.php  and select
    //       1 minute intvl
    //       moon
    //       Bolton Mass
    //    This will provide a table of moon azimuth and elevation for the
    //    selected location, in 1 minute intervals.
    //    For example:
    //        Eastern Standard Time
    //                   Altitude    Azimuth    Fraction                                     
    //                               (E of N)  Illuminated
    //        h  m         o           o   
    //       20:49       45.8       105.7       1.00
    //       20:50       46.0       105.9       1.00
    //       20:51       46.2       106.2       1.00
    //       20:52       46.3       106.4       1.00 
    //       20:53       46.5       106.6       1.00 
    //       20:54       46.7       106.8       1.00
    // 2) Next run tcx, starting it near the begining of a minute.
    //       sky_init: LCL now = 2019-Jan-20 20:52:00 EST
    //       unit_test: NAME         RA        DEC        MAG         AZ         EL
    //       unit_test: Moon   121.1609    20.1905      -12.9   106.5391    46.5423
    // RESULTS)
    //    At 2019-Jan-20 20:52:00 EST:
    //    - tcx program moon az/el   106.54  46.54
    //    - usno moon az/el          106.4   46.3
    //    - difference                 0.14   0.24
    // OBSERVATIONS) 
    //    - the result from tcx most closely match the usno position
    //      one minute later, at 20:53

    // restore latite/longitude
    latitude = lat_save;
    longitude = long_save;

    INFO2("tests passed\n");
}

static bool is_close(double act, double exp, double allowed_deviation, double * deviation)
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
