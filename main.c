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
    char *incl_ss_obj_str = NULL;

    // get latitude and longitude from env variables TCX_LAT and TCX_LONG;
    // notes
    // - east longitude is positive
    // - solar_sys_data.csv should be created using a Horizons observer site code
    //   near the TCX_LAT,TCX_LONG; if this is not done the position of the Moon could
    //   be in error by as much as 2 degrees
    // - Horizons Observer Site Code seems to include observatory codes, refer to
    //   https://en.wikipedia.org/wiki/List_of_observatory_codes
    // - my location is specified as follows in .bash_profile
    //     # from https://www.latlong.net for Bolton Mass USA
    //     export TCX_LAT=42.422986
    //     export TCX_LONG=-71.623798
    lat_str = getenv("TCX_LAT");
    long_str = getenv("TCX_LONG");
    if (lat_str == NULL || long_str == NULL) {
        FATAL("latitude and longitude must be supplied in environment variables TCX_LAT and TCX_LONG\n");
    }
    if (sscanf(lat_str, "%lf", &latitude) != 1 || latitude < -90 || latitude > 90) {
        FATAL("invalid latitude '%s' \n", lat_str);
    }
    if (sscanf(long_str, "%lf", &longitude) != 1 || longitude < -180 || longitude > 180) {
        FATAL("invalid longitude '%s' \n", lat_str);
    }
    INFO("latitude = %0.6lf  longitude = %0.6lf\n", latitude, longitude);

    // get and process options
    while (true) {
        char opt_char = getopt(argc, argv, "i:");
        if (opt_char == -1) {
            break;
        }
        switch (opt_char) {
        case 'i':
            incl_ss_obj_str = optarg;
            break;
        default:
            return 1;
        }
    }
    
    // initialize sky module
    ret = sky_init(incl_ss_obj_str);
    if (ret < 0) {
        FATAL("sky_init ret %d\n", ret);
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
    int sky_view_pane_x, sky_view_pane_y, sky_view_pane_w, sky_view_pane_h;

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

    sky_view_pane_x = sky_ctl_pane_w;
    sky_view_pane_y = sky_pane_h;  
    sky_view_pane_w = win_height - sky_pane_h;
    sky_view_pane_h = win_height - sky_pane_h;
    INFO("sky_view_pane x,y,w,h = %d %d %d %d\n", sky_view_pane_x, sky_view_pane_y, sky_view_pane_w, sky_view_pane_h);

    sdl_pane_manager(
        NULL,           // context
        NULL,           // called prior to pane handlers
        NULL,           // called after pane handlers
        100000,         // 0=continuous, -1=never, else us 
        3,              // number of pane handler varargs that follow
        sky_pane_hndlr, NULL, sky_pane_x, sky_pane_y, sky_pane_w, sky_pane_h, PANE_BORDER_STYLE_MINIMAL,
        sky_ctl_pane_hndlr, NULL, sky_ctl_pane_x, sky_ctl_pane_y, sky_ctl_pane_w, sky_ctl_pane_h, PANE_BORDER_STYLE_MINIMAL,
        sky_view_pane_hndlr, NULL, sky_view_pane_x, sky_view_pane_y, sky_view_pane_w, sky_view_pane_h, PANE_BORDER_STYLE_MINIMAL
                        );
}
