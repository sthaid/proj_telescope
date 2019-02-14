#ifndef __UTIL_SKY_H__
#define __UTIL_SKY_H__

//
// defines
//

#define OBJTYPE_NONE       0
#define OBJTYPE_STELLAR    1
#define OBJTYPE_SOLAR      2
#define OBJTYPE_PLACE_MARK 3

#define NO_VALUE     9999
#define NO_VALUE_STR "9999"

//
// typedefs
//

//
// variables
//

int max_obj;

//
// prototypes
//

// init
int util_sky_init(char *incl_obj_str, bool run_unit_test);

// get an object
int get_obj(int i, time_t t, double lst, char **name, int *type, double *ra, double *dec, double *mag, double *az, double *el);

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
void hr2hms(double hr, int * hour, int * minute, double * seconds);
char * gmtime_str(time_t t, char *str);
char *localtime_str(time_t t, char *str);
char * hr_str(double hr, char *str);

// unit test
void util_sky_unit_test(void);

#endif
