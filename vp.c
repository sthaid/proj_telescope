/*
Copyright (c) 2019 Steven Haid

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
#include "util_sky.h"

//
// defines
//

#define MAX_OBJ    16
#define MAX_INTVL 100

#undef FATAL 
#define FATAL(fmt, args...) \
    do { \
        printf("FATAL: " fmt, ## args); \
        exit(1); \
    } while (0)

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
    bool in_range;
} obj_t;

//
// variables
//

//
// prototypes
//

void usage(void);
void get_lat_long_from_env(void);
void get_next_day(int *m, int *d, int *y);
void get_start_and_end_times(int m, int d, int y, time_t *ts, time_t *te);
int check_start_and_end_times(char *start_t_str, char *end_t_str);

// -----------------  MAIN - VIEW PLANNER  --------------------------------

int main(int argc, char ** argv)
{
    double min_az, max_az, min_el, max_el;         // args
    char *start_time, *end_time, *obj_name_list;
    int max_day;

    int month, day, year;                          // today's date

    // refer to usage() proc for description of args
    if (argc != 9) {
        usage();
        return 1;
    }
    if (sscanf(argv[1], "%lf", &min_az) != 1 ||
        sscanf(argv[2], "%lf", &max_az) != 1 ||
        sscanf(argv[3], "%lf", &min_el) != 1 ||
        sscanf(argv[4], "%lf", &max_el) != 1 ||
        sscanf(argv[7], "%d",  &max_day) != 1)
    {
        usage();
        return 1;
    }
    start_time = argv[5];
    end_time = argv[6];
    obj_name_list = argv[8];

    // check args
    if (min_az < 0 || min_az > 360) {
        FATAL("invalid min_az %g\n", min_az);
    }
    if (max_az < 0 || max_az > 360) {
        FATAL("invalid max_az %g\n", max_az);
    }
    if (min_el < 0 || min_el > 90) {
        FATAL("invalid min_el %g\n", min_el);
    }
    if (max_el < 0 || max_el > 90) {
        FATAL("invalid max_el %g\n", max_el);
    }
    if (min_el >= max_el) {
        FATAL("invalid min_el %g, max_el %g\n", min_el, max_el);
    }
    if (check_start_and_end_times(start_time, end_time) < 0) {
        FATAL("invalid start_time='%s' or end_time='%s'\n", start_time, end_time);
    }
    if (max_day <= 0) {
        FATAL("max_day %d\n", max_day);
    }

    // get latitude and logitude from environment vars TCX_LAT and TCX_LONG
    get_lat_long_from_env();

    // call util_sky_init, and verify not too many objs
    if (util_sky_init(obj_name_list, false, false) < 0) {
        FATAL("sky_init failed\n");
    }
    if (max_obj > MAX_OBJ) {
        FATAL("max_obj=%d too large\n", max_obj);
    }

    // get today's month,day,year
    {
    time_t t = time(NULL);
    struct tm * tm = localtime(&t);
    month = tm->tm_mon+1;
    day   = tm->tm_mday;
    year  = tm->tm_year+1900;
    }

    // print program args
    {
    char util_sky_obj_name_list[1000], *p;
    int i;
    p = util_sky_obj_name_list;
    for (i = 0; i < max_obj; i++) {
        p += sprintf(p, "%s ", get_obj_name(i));
    }
    printf("az_range   = %g %g\n", min_az, max_az);
    printf("el_range   = %g %g\n", min_el, max_el);
    printf("start_time = %s\n", start_time);
    printf("end_time   = %s\n", end_time);
    printf("max_day    = %d\n", max_day);
    printf("lat/long   = %0.6lf %0.6lf\n", latitude, longitude);
    printf("max_obj    = %d : %s\n", max_obj, util_sky_obj_name_list);
    printf("start_date = %d/%d/%d  (m/d/y)\n", month, day, year);
    printf("\n");
    }

    // loop over days
    // - for each day get the azel of each object over the time interval;
    //   and determine if/when each object is in range
    // - print results which are the az/el of the objects that are in range,
    //   and the time that they are in range
    int d;
    for (d = 0; d < max_day; d++) {
        bool one_or_more_obj_in_range, obj_in_range[MAX_OBJ];
        double lst[MAX_INTVL];
        time_t t[MAX_INTVL];
        obj_t obj[MAX_INTVL][MAX_OBJ];
        int n_interval;
        double t_interval;
        time_t t_start, t_end;
        int n, i;
        char *p, str[1000];

        // advance to the next day
        if (d > 0) {
            get_next_day(&month, &day, &year);
        }

        // get start and end times;
        // striving for an interval of about 1 hour:
        // - determine the number of intervals that comprise the time range, and
        // - determine the length of the time interval
        get_start_and_end_times(month, day, year, &t_start, &t_end);
        n_interval = (t_end - t_start) / 3600;
        t_interval = (t_end - t_start) / n_interval;

        // for this day, get each object's az/el for the times that comprese the time range
        // - for each time and object determine if it is in range, setting obj[n][i].in_range flag
        // - set one_or_more_obj_in_range flag if any object is in range for any time
        // - set obj_in_range[i] flag if obj 'i' is in range for any time
        one_or_more_obj_in_range = false;
        memset(obj_in_range, 0, sizeof(obj_in_range));
        for (n = 0; n <= n_interval; n++) {
            t[n] = t_start + t_interval * n;
            lst[n] = ct2lst(longitude, jdconv2(t[n]));

            for (i = 0; i < max_obj; i++) {
                bool el_in_range, az_in_range;

                get_obj(i, t[n], lst[n], 
                        &obj[n][i].name, 
                        &obj[n][i].type, 
                        &obj[n][i].ra, 
                        &obj[n][i].dec, 
                        &obj[n][i].mag, 
                        &obj[n][i].az, 
                        &obj[n][i].el);

                el_in_range = (obj[n][i].el >= min_el && obj[n][i].el <= max_el);
                if (max_az >= min_az) {
                    az_in_range = (obj[n][i].az >= min_az && obj[n][i].az <= max_az);
                } else {
                    az_in_range = (obj[n][i].az >= min_az || obj[n][i].az <= max_az);
                }

                obj[n][i].in_range = (az_in_range && el_in_range);

                if (obj[n][i].in_range) {
                    obj_in_range[i] = true;
                    one_or_more_obj_in_range = true;
                }
            }
        }

        // for this day: 
        // if none of the objects are in range at any time
        // then there is nothing to print, so continue
        if (one_or_more_obj_in_range == false) {
            continue;
        }

        // for this day: 
        // print the az/el of the objects that are in range for each time 

        // - construct and print the header line, which contains the object name of 
        //   each of the objects that are in range for one or more times
        p = str;
        p += sprintf(p, "%17s ", "");
        for (i = 0; i < max_obj; i++) {
            char name_str[100];

            if (!obj_in_range[i]) {
                continue;
            }
            strncpy(name_str, obj[0][i].name, 8);
            name_str[8] = '\0';
            p += sprintf(p, "%8s ", name_str);
        }
        printf("%s\n", str);

        // - construct and print lines for each time; if a time has no objects in range 
        //   then do not print a line for that time
        for (n = 0; n <= n_interval; n++) {
            char azel_str[100];
            char time_str[100];

            // if no objects are in range for this time then continue
            for (i = 0; i < max_obj; i++) {
                if (obj[n][i].in_range) {
                    break;
                }
            }
            if (i == max_obj) {
                continue;
            }

            // construct the string
            // - starting with the time, and
            // - followed by the azel of each object that has been identified 
            //   as being in range for one or more times
            p = str;
            p += sprintf(p, "%17s ", time2str(time_str, t[n]*1000000, false, false, true));
            for (i = 0; i < max_obj; i++) {
                if (!obj_in_range[i]) {
                    continue;
                }

                if (obj[n][i].in_range) {
                    sprintf(azel_str, "%3ld %2ld", lrint(obj[n][i].az), lrint(obj[n][i].el));
                    if (strlen(azel_str) > 8) {
                        FATAL("azel_str '%s' too long\n", azel_str);
                    }
                } else {
                    azel_str[0] = '\0';
                }
                p += sprintf(p, "%8s ", azel_str);
            }
            printf("%s\n", str);
        }
        printf("\n");
    }

    // done
    return 0;
}

// -----------------  SUPPORT ROUTINES  -----------------------------------

void usage(void)
{
    printf("\
\n\
usage: vp min_az max_az min_el max_el start_time end_time max_day obj_name,...\n\
\n\
examples of 'start_time end_time'\n\
  18 23              : 18:00 to 23:00\n\
  18 2               : 18:00 to 02:00 \n\
  sunset sunrise     : sunset to sunrise\n\
  sunset+1  sunset+5 : one hour after sunset to 5 hours after sunset\n\
  sunrise-4 sunrise  : four hours before sunrise to sunrise\n\
\n\
example of cmdline:\n\
  vp 135 225 20 90 sunset sunset+5 365 saturn,jupiter\n\
will provide dates and times to view saturn and jupiter at\n\
azimuth 135 to 225 (clockwise), elevation 20 to 90 amd from \n\
sunset to 5 hours after sunset; over the next 365 days\n\
\n\
");
}

void get_lat_long_from_env(void)
{
    char *lat_str, *long_str;

    lat_str = getenv("TCX_LAT");
    long_str = getenv("TCX_LONG");
    if (lat_str == NULL || long_str == NULL) {
        FATAL("latitude and longitude must be supplied in environment variables TCX_LAT and TCX_LONG\n");
    }
    if (sscanf(lat_str, "%lf", &latitude) != 1 || latitude < -90 || latitude > 90) {
        FATAL("invalid TCX_LAT '%s' \n", lat_str);
    }
    if (sscanf(long_str, "%lf", &longitude) != 1 || longitude < -180 || longitude > 180) {
        FATAL("invalid TCX_LONG '%s' \n", long_str);
    }
}

void get_next_day(int *m, int *d, int *y)
{                                   //   Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
    static char days_in_month[13] = { 0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

    days_in_month[2] = ((*y % 4) != 0) ? 28 : 29;  // not correct for 2100

    if (*d < days_in_month[*m]) {
        *d = *d + 1;
    } else {
        *d = 1;
        if (*m < 12) {
            *m = *m + 1;
        } else {
            *m = 1;
            *y = *y + 1;
        }
    }
}

// XXX work here
void get_start_and_end_times(int m, int d, int y, time_t *ts, time_t *te)
{
// XXX time range based on sunset and sunrise
    struct tm tm;

    memset(&tm, 0, sizeof(tm));
    tm.tm_mon = m - 1;
    tm.tm_year = y - 1900;
    tm.tm_mday = d;
    tm.tm_hour =  18;
    tm.tm_isdst = -1;
    *ts = mktime(&tm);

    memset(&tm, 0, sizeof(tm));
    tm.tm_mon = m - 1;
    tm.tm_year = y - 1900;
    tm.tm_mday = d+1;
    tm.tm_hour =  7;
    tm.tm_isdst = -1;
    *te = mktime(&tm);
}

int check_start_and_end_times(char *start_t_str, char *end_t_str)
{
    return 0;
}
