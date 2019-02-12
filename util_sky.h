#ifndef __UTIL_SKY_H__
#define __UTIL_SKY_H__

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
    double x;  // XXX put these in diff struct
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

//
// prototypes
//

// init
int util_sky_init(char *incl_ss_obj_str);

// convert ra,dec to/from az,el
double jdconv(int yr, int mn, int day, double hour);
double jdconv2(time_t t);
void jd2ymdh(double jd, int *year, int *month, int *day, double *hour);
double ct2lst(double lng, double jd);
void radec2azel(double *az, double *el, double ra, double dec, double lst, double lat);
void azel2radec(double *ra, double *dec, double az, double el, double lst, double lat);

// sunrise and sunset times
void sunrise_sunset(double jd, time_t *trise, time_t *tset);

// misc
int compute_ss_obj_ra_dec_mag(obj_t *x, time_t t);
void hr2hms(double hr, int * hour, int * minute, double * seconds);
char * gmtime_str(time_t t, char *str);
char *localtime_str(time_t t, char *str);
char * hr_str(double hr, char *str);

// unit test
void util_sky_unit_test(void);

#endif
