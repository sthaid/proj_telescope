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

#define MAX_OBJ  16
#define MAX_HOUR 24

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

// XXX DST problem

// -----------------  MAIN - VIEW PLANNER  --------------------------------

int main(int argc, char ** argv)
{
    double min_az, max_az, min_el, max_el;
    int start_hour, max_hour, max_day;
    char *obj_name_list, *p, util_sky_obj_name_list[1000], time_str[100];
    int ret, d, h, i;
    time_t t_start;
    struct tm tm;

    // get args
    // usage: vp min_az max_az min_el mazel start_hours max_hour max_day obj_name_list  
    if (argc != 9) {
        usage();
    }
    if (sscanf(argv[1], "%lf", &min_az) != 1 ||
        sscanf(argv[2], "%lf", &max_az) != 1 ||
        sscanf(argv[3], "%lf", &min_el) != 1 ||
        sscanf(argv[4], "%lf", &max_el) != 1 ||
        sscanf(argv[5], "%d", &start_hour) != 1 ||
        sscanf(argv[6], "%d", &max_hour) != 1 ||
        sscanf(argv[7], "%d", &max_day) != 1)
    {
        usage();
    }
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
    if (start_hour < 0 || start_hour > 23) {
        FATAL("start_hour %d\n", start_hour);
    }
    if (max_hour < 0 || max_hour > 24) {
        FATAL("max_hour %d\n", max_hour);
    }
    if (max_day <= 0) {
        FATAL("max_day %d\n", max_day);
    }

    // get latitude and logitude from environment vars
    get_lat_long_from_env();

    // util_sky_init
    ret = util_sky_init(obj_name_list, false, false);
    if (ret < 0) {
        FATAL("sky_init ret %d\n", ret);
    }

    // checks
    if (max_obj > MAX_OBJ) {
        FATAL("max_obj=%d too large\n", max_obj);
    }

    // get start time, by modifying current local time to start_hour
    t_start = time(NULL);
    tm = *localtime(&t_start);
    tm.tm_hour = start_hour;
    tm.tm_min  = 0;
    tm.tm_sec  = 0;
    t_start = timelocal(&tm);

    // make list of objects that util_sky_init identified 
    p = util_sky_obj_name_list;
    for (i = 0; i < max_obj; i++) {
        p += sprintf(p, "%s ", get_obj_name(i));
    }

    // prints 
    printf("az_range   = %g %g\n", min_az, max_az);
    printf("el_range   = %g %g\n", min_el, max_el);
    printf("start_hour = %d\n", start_hour);
    printf("hours      = %d\n", max_hour);
    printf("days       = %d\n", max_day);
    printf("lat/long   = %0.6lf %0.6lf\n", latitude, longitude);
    printf("max_obj    = %d : %s\n", max_obj, util_sky_obj_name_list);
    printf("start_time = %s\n", time2str(time_str, t_start*1000000L, false, false, true));
    printf("\n");

    // loop over days
    for (d = 0; d < max_day; d++) {
        bool one_or_more_obj_in_range, obj_in_range[MAX_OBJ];
        double lst[MAX_HOUR];
        time_t t[MAX_HOUR];
        obj_t obj[MAX_HOUR][MAX_OBJ];

        // for this day, get each object az/el for the range of hours, and
        // - for each hour and object determine if it is in range
        // - set one_or_more_obj_in_range flag if any object is in range for any hour
        // - set obj_in_range[i] flag if obj 'i' is in range for any hour
        one_or_more_obj_in_range = false;
        memset(obj_in_range, 0, sizeof(obj_in_range));
        for (h = 0; h < max_hour; h++) {
            t[h] = t_start + d * 86400 + h * 3600;
            lst[h] = ct2lst(longitude, jdconv2(t[h]));

            for (i = 0; i < max_obj; i++) {
                bool el_in_range, az_in_range;

                get_obj(i, t[h], lst[h], 
                        &obj[h][i].name, 
                        &obj[h][i].type, 
                        &obj[h][i].ra, 
                        &obj[h][i].dec, 
                        &obj[h][i].mag, 
                        &obj[h][i].az, 
                        &obj[h][i].el);

                el_in_range = (obj[h][i].el >= min_el && obj[h][i].el <= max_el);
                if (max_az >= min_az) {
                    az_in_range = (obj[h][i].az >= min_az && obj[h][i].az <= max_az);
                } else {
                    az_in_range = (obj[h][i].az >= min_az || obj[h][i].az <= max_az);
                }

                obj[h][i].in_range = (az_in_range && el_in_range);

                if (obj[h][i].in_range) {
                    obj_in_range[i] = true;
                    one_or_more_obj_in_range = true;
                }
            }
        }

        // for this day: 
        // if none of the objects are in range at any hour
        // then there is nothing worth printing, so continue
        if (one_or_more_obj_in_range == false) {
            continue;
        }

        // for this day: 
        // print the az/el of the objects that are in range ...
        char *p, str[1000], name_str[100], azel_str[100], mdy_str[100];
        double hour;

        // construct and print the header line, which contains the date and
        // the object name for each of the objects that are in range for one or more hours
        tm = *localtime(&t[0]);
        sprintf(mdy_str, "%d/%d/%d", tm.tm_mon+1, tm.tm_mday, tm.tm_year+1900);
        p = str;
        p += sprintf(p, "%-10s ", mdy_str);
        for (i = 0; i < max_obj; i++) {
            if (!obj_in_range[i]) {
                continue;
            }
            strncpy(name_str, obj[0][i].name, 8);
            name_str[8] = '\0';
            p += sprintf(p, "%8s ", name_str);
        }
        printf("%s\n", str);

        // construct and print lines for each hour; if an hour has no objects
        // in range then do not print a line for that hour
        for (h = 0; h < max_hour; h++) {
#if 1  // XXX debug dst problem here
            // if no objects are in range for this hour then continue
            for (i = 0; i < max_obj; i++) {
                if (obj[h][i].in_range) {
                    break;
                }
            }
            if (i == max_obj) {
                continue;
            }
#endif

            // determine the hour, it is determined in floating point however
            // the hour is supposed to be an integer
            tm = *localtime(&t[h]);
            hour = tm.tm_hour + tm.tm_min / 60. + tm.tm_sec / 3600.;

            // construct the string
            // - starting with the hour, and
            // - followed by the azel of each object that has been identified 
            //   as being in range for one or more hours
            p = str;
            p += sprintf(p, "%-10g ", hour);
            for (i = 0; i < max_obj; i++) {
                if (!obj_in_range[i]) {
                    continue;
                }

                if (obj[h][i].in_range) {
                    sprintf(azel_str, "%3ld %2ld", lrint(obj[h][i].az), lrint(obj[h][i].el));
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

void usage(void)
{
    FATAL("usage: vp min_az max_az min_el max_el start_hour max_hour max_day obj_name,...\n");
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
