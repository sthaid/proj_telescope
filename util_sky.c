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

#include "common.h"
#include "util_sky.h"

//
// defines
//

#define SIND(x)   (sin((x)*DEG2RAD))
#define COSD(x)   (cos((x)*DEG2RAD))
#define TAND(x)   (tan((x)*DEG2RAD))
#define ACOSD(x)  (acos(x)*RAD2DEG)
#define ASIND(x)  (asin(x)*RAD2DEG)

#define RAD2DEG (180. / M_PI)
#define DEG2RAD (M_PI / 180.)
#define HR2RAD  (M_PI / 12.)

#define JD2000 2451545.0

//
// typedefs
//

//
// prototypes
//

bool is_close(double act, double exp, double allowed_deviation, double * deviation);

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
    if (M < 0) FATAL("M < 0\n");
    while (M > 360) M -= 360; 

    // equation of the center
    C = 1.9148 * SIND(M) + 0.0200 * SIND(2*M) + 0.0003 * SIND(3*M);

    // ecliptic longitude
    lambda = (M +  C + 180 + 102.9372);
    if (lambda < 0) FATAL("lambda < 0\n");
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

void hr2hms(double hr, int * hour, int * minute, double * seconds)
{
    double secs = hr * 3600;

    *hour = secs / 3600;
    secs -= 3600 * *hour;
    *minute = secs / 60;
    secs -= *minute * 60;
    *seconds = secs;
}

// -----------------  TEST ALGORITHMS  ------------------------------------

void util_sky_unit_test(void) 
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
        FATAL("az_deviation = %f, exceeds limit\n", az_deviation);
    }
    if (!is_close(el, el_exp, 0.00001, &el_deviation)) {
        FATAL("el_deviation = %f, exceeds limit\n", el_deviation);
    }
    INFO("az_deviation = %f  el_deviation = %f\n", az_deviation, el_deviation);
    }

#if 0 // XXX move
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
    INFO("            NAME         RA        DEC        MAG         AZ         EL\n");
    for (i = 0; i < max_obj; i++) {
        obj_t *x = &obj[i];

        if (x->type != OBJTYPE_SOLAR) {
            continue;
        }

        compute_ss_obj_ra_dec_mag(x, t);
        radec2azel(&az, &el, x->ra, x->dec, lst, latitude);

        INFO("%16s %10.4f %10.4f %10.1f %10.4f %10.4f\n", x->name, x->ra, x->dec, x->mag, az, el);
    } }

    // time how long to convert all stellar objects to az/el
    { double lst, az, el;
      time_t t;
      int i;
      unsigned long start_us, duration_us;
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
    INFO("radec2azel perf: %d objects in %ld ms\n", max_obj, duration_us/1000);
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
            FATAL("objname %s: radec %f %f -> azel %f %f -> radec %f %f (deviation %f %f)\n",
                  x->name, x->ra, x->dec, az, el, ra, dec,
                  ra_deviation, dec_deviation);
        }
    } }
#endif

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
