// XXX vvv fix this file
// convert ra,dec to az,el
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
// unit test
void util_sky_unit_test(void);

