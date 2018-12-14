#include "common.h"

//
// defines
//

#define DEFAULT_WIN_WIDTH  1920
#define DEFAULT_WIN_HEIGHT 1080

//
// typedefs
//

//
// variables
//

//
// prototypes
//

void display_handler(void);

// -----------------  MAIN  -----------------------------------------------

int main(int argc, char ** argv)
{
    int ret;
    char *lat_str, *long_str;

    // get latitude and longitude from env variables MY_LAT and MY_LONG;
    // notes
    // - east longitude is positive
    // - solar_sys_data.csv should be created using a Horizons observer site code
    //   near the MY_LAT,MY_LONG; if this is not done the position of the Moon could
    //   be in error by as much as 2 degrees
    // - Horizons Observer Site Code seems to include observatory codes, refer to
    //   https://en.wikipedia.org/wiki/List_of_observatory_codes
    // - my location is specified as follows in .bash_profile
    //     # from https://www.latlong.net for Bolton Mass USA
    //     export MY_LAT=42.422986
    //     export MY_LONG=-71.623798
    lat_str = getenv("MY_LAT");
    long_str = getenv("MY_LONG");
    if (lat_str == NULL || long_str == NULL) {
        ERROR("latitude and longitude must be supplied in environment variables MY_LAT and MY_LONG\n");
    }
    if (sscanf(lat_str, "%lf", &latitude) != 1 || latitude < -90 || latitude > 90) {
        ERROR("invalid latitude '%s' \n", lat_str);
    }
    if (sscanf(long_str, "%lf", &longitude) != 1 || longitude < -180 || longitude > 180) {
        ERROR("invalid longitude '%s' \n", lat_str);
    }
    INFO("latitude = %0.6lf  longitude = %0.6lf\n", latitude, longitude);
    
    // initialize sky module
    ret = sky_init();
    if (ret < 0) {
        return ret;
    }

    // draw display and handle user inputs
    display_handler();

    // done
    return 0;
}

void display_handler(void)
{
    int win_width  = DEFAULT_WIN_WIDTH;
    int win_height = DEFAULT_WIN_HEIGHT;
    int sky_pane_x, sky_pane_y, sky_pane_w, sky_pane_h;
    int sky_ctl_pane_x, sky_ctl_pane_y, sky_ctl_pane_w, sky_ctl_pane_h;

    if (sdl_init(&win_width, &win_height, true) < 0) {
        FATAL("sdl_init %dx%d failed\n", win_width, win_height);
    }
    INFO("win_width=%d win_height=%d\n", win_width, win_height);

    sky_pane_x     = 0;
    sky_pane_y     = 0;
    sky_pane_w     = win_width;
    sky_pane_h     = win_height / 2;
    INFO("sky_pane x,y,w,h = %d %d %d %d\n", sky_pane_x, sky_pane_y, sky_pane_w, sky_pane_h);

    sky_ctl_pane_x = 0;
    sky_ctl_pane_y = sky_pane_h;;
    sky_ctl_pane_w = 420  ;
    sky_ctl_pane_h = win_height - sky_pane_h;
    INFO("sky_ctl_pane x,y,w,h = %d %d %d %d\n", sky_ctl_pane_x, sky_ctl_pane_y, sky_ctl_pane_w, sky_ctl_pane_h);

    sdl_pane_manager(
        NULL,           // context
        NULL,           // called prior to pane handlers
        NULL,           // called after pane handlers
        1000000,        // 0=continuous, -1=never, else us 
        2,              // number of pane handler varargs that follow
        sky_pane_hndlr, NULL, sky_pane_x, sky_pane_y, sky_pane_w, sky_pane_h, PANE_BORDER_STYLE_MINIMAL,
        sky_ctl_pane_hndlr, NULL, sky_ctl_pane_x, sky_ctl_pane_y, sky_ctl_pane_w, sky_ctl_pane_h, PANE_BORDER_STYLE_MINIMAL
                        );
}
