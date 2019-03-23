// build:  gcc -o az -Wall -lm az.c
//
// Google maps
//   deck:        42.422224  -71.622796
//   smith barn   42.422907  -71.623862
// From https://www.movable-type.co.uk/scripts/latlong.html
//   distance  .1159 km     = 380.25 feet
//   bearing   310 57 24    = 310.96 degrees
//
// This program result:
//
//   $ ./az
//   DECK->BARN
//     distance = 379.87 feet
//     azimuth  = 310.96 degrees
//   
//   HOUSE SOUTH->NORTH
//     distance = 69.88 feet
//     azimuth  = 313.93 degrees

#include <stdio.h>
#include <math.h>

#define RAD2DEG (180. / M_PI)
#define DEG2RAD (M_PI / 180.)

#define SIND(x)      (sin((x)*DEG2RAD))
#define COSD(x)      (cos((x)*DEG2RAD))
#define TAND(x)      (tan((x)*DEG2RAD))
#define ACOSD(x)     (acos(x)*RAD2DEG)
#define ASIND(x)     (asin(x)*RAD2DEG)
#define ATAN2D(y,x)  (atan2(y,x)*RAD2DEG)

#define FT_PER_NM  6076.12

typedef struct {
    double lat;
    double longi;
} point_t;

void calc_az_and_dist(point_t *p1, point_t *p2, char *str);

int main(int argc, char **argv)
{
    point_t deck = {42.422224, -71.622796};
    point_t barn = {42.422907, -71.623862};
    calc_az_and_dist(&deck, &barn, "DECK->BARN");

    point_t house_south = {42.422086, -71.622592};
    point_t house_north = {42.422219, -71.622779};
    calc_az_and_dist(&house_south, &house_north, "HOUSE SOUTH->NORTH");

    return 0;
}

void calc_az_and_dist(point_t *p1, point_t *p2, char *str)
{
    double az, dlat, dlong, dist;

    dlat  = (p2->lat - p1->lat);
    dlong = (p2->longi - p1->longi) * COSD(p1->lat);

    dist = sqrt(dlat*dlat + dlong*dlong) * 60 * FT_PER_NM;
    az   = ATAN2D(dlong,dlat);
    if (az < 0) az += 360;

    printf("%s\n", str);
    printf("  distance = %0.2lf feet\n", dist);
    printf("  azimuth  = %0.2lf degrees\n", az);
    printf("\n");
}
