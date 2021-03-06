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

static void display_handler(void);

// -----------------  MAIN  -----------------------------------------------

int main(int argc, char ** argv)
{
    int ret;
    char *lat_str, *long_str;
    char *az_tele_leg_1_str;
    char *incl_obj_str = NULL;

    // get latitude and longitude from env variables TCX_LAT and TCX_LONG;
    // notes
    // - east longitude is positive
    // - solar_sys_data.csv should be created using a Horizons observer site code
    //   near the TCX_LAT,TCX_LONG; if this is not done the position of the Moon could
    //   be in error by as much as 2 degrees
    // - Horizons Observer Site Code seems to include observatory codes, refer to
    //   https://en.wikipedia.org/wiki/List_of_observatory_codes
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
    INFO("latitude = %0.6lf  longitude = %0.6lf\n", latitude, longitude);

    // get ctlr_ip env variable
    ctlr_ip = getenv("TCX_CTLR_IP");
    if (ctlr_ip == NULL) {
        FATAL("ctlr_ip must be supplied in environment variables TCX_CTLR_IP\n");
    }
    INFO("ctlr_ip = %s\n", ctlr_ip);

    // get az_tele_leg_1: the azimuth of telescoe leg 1
    az_tele_leg_1_str = getenv("TCX_AZ_TELE_LEG_1");
    if (az_tele_leg_1_str == NULL) {
        FATAL("TCX_AZ_TELE_LEG_1 must be supplied in environment variables\n");
    }
    if (sscanf(az_tele_leg_1_str, "%lf", &az_tele_leg_1) != 1) {
        FATAL("invalid TCX_AZ_TELE_LEG_1 '%s'\n", az_tele_leg_1_str);
    }
    INFO("az_tele_leg_1 = %0.2lf\n", az_tele_leg_1);

    // get and process options
    while (true) {
        char opt_char = getopt(argc, argv, "i:");
        if (opt_char == -1) {
            break;
        }
        switch (opt_char) {
        case 'i':
            incl_obj_str = optarg;
            break;
        default:
            return 1;
        }
    }
    
    // initialize 
    ret = sky_init(incl_obj_str);
    if (ret < 0) {
        FATAL("sky_init ret %d\n", ret);
    }
    ret = tele_init();
    if (ret < 0) {
        FATAL("tele_init ret %d\n", ret);
    }

    // draw display and handle user inputs
    display_handler();

    // done
    return 0;
}

static void display_handler(void)
{
    int win_width, win_height;
    int sky_pane_x, sky_pane_y, sky_pane_w, sky_pane_h;
    int tele_pane_x, tele_pane_y, tele_pane_w, tele_pane_h;
    int multi_pane_x, multi_pane_y, multi_pane_w, multi_pane_h;

    // init sdl, and get actual window width and height
    win_width  = DEFAULT_WIN_WIDTH;
    win_height = DEFAULT_WIN_HEIGHT;
    if (sdl_init(&win_width, &win_height, true) < 0) {
        FATAL("sdl_init %dx%d failed\n", win_width, win_height);
    }
    INFO("REQUESTED win_width=%d win_height=%d\n", DEFAULT_WIN_WIDTH, DEFAULT_WIN_HEIGHT);
    INFO("ACTUAL    win_width=%d win_height=%d\n", win_width, win_height);

    // sky_pane coordinates
    sky_pane_x     = 0;
    sky_pane_y     = 0;
    sky_pane_w     = win_width;
    if (win_height < 900) {
        sky_pane_h = win_height - 480;
    } else {
        sky_pane_h = win_height - 640;
    }
    INFO("sky_pane x,y,w,h = %d %d %d %d\n", sky_pane_x, sky_pane_y, sky_pane_w, sky_pane_h);

    // tele_pane coordinates
    tele_pane_x = 0;
    tele_pane_y = sky_pane_h;  
    tele_pane_h = win_height - sky_pane_h;
    tele_pane_w = tele_pane_h * (640. / 480.);
    INFO("tele_pane x,y,w,h = %d %d %d %d\n", tele_pane_x, tele_pane_y, tele_pane_w, tele_pane_h);

    // the multi_pane coordinates are used for all of the following panes:
    // - sky_view_pane
    // - tele_motor_info_pane
    // - tele_cam_info_pane
    multi_pane_x = tele_pane_x + tele_pane_w;
    multi_pane_y = tele_pane_y;
    multi_pane_h = tele_pane_h;
    multi_pane_w = multi_pane_h;
    INFO("multi_pane x,y,w,h = %d %d %d %d\n", multi_pane_x, multi_pane_y, multi_pane_w, multi_pane_h);

    // call the pane manger; 
    // this will not return except when it is time to terminate the program
    sdl_pane_manager(
        NULL,           // context
        NULL,           // called prior to pane handlers
        NULL,           // called after pane handlers
        100000,         // 0=continuous, -1=never, else us 
        5,              // number of pane handler varargs that follow
        sky_pane_hndlr, NULL, sky_pane_x, sky_pane_y, sky_pane_w, sky_pane_h, PANE_BORDER_STYLE_MINIMAL,
        tele_pane_hndlr, NULL, tele_pane_x, tele_pane_y, tele_pane_w, tele_pane_h, PANE_BORDER_STYLE_MINIMAL,
        tele_motor_info_pane_hndlr, NULL, multi_pane_x, multi_pane_y, multi_pane_w, multi_pane_h, PANE_BORDER_STYLE_MINIMAL,
        tele_cam_info_pane_hndlr, NULL, multi_pane_x, multi_pane_y, multi_pane_w, multi_pane_h, PANE_BORDER_STYLE_MINIMAL,
        sky_view_pane_hndlr, NULL, multi_pane_x, multi_pane_y, multi_pane_w, multi_pane_h, PANE_BORDER_STYLE_MINIMAL
                        );
}
