#include "common.h"

//
// defines
//

#define DEFAULT_WIN_WIDTH  1900
#define DEFAULT_WIN_HEIGHT 1000

// XXX win_width=1920 win_height=981
#define PH_SKY_X  0
#define PH_SKY_Y  0
#define PH_SKY_W  1500
#define PH_SKY_H  500 

#define PH_SKY_CTL_X  1500
#define PH_SKY_CTL_Y  0
#define PH_SKY_CTL_W  420  
#define PH_SKY_CTL_H  500 

//
// typedefs
//

//
// variables
//

int win_width;
int win_height;

//
// prototypes
//

void display_handler(void);

// -----------------  MAIN  -----------------------------------------------

int main(int argc, char ** argv)
{
    int ret;

    ret = sky_init();
    if (ret < 0) {
        return ret;
    }

    display_handler();
}

void display_handler(void)
{
    win_width  = DEFAULT_WIN_WIDTH;
    win_height = DEFAULT_WIN_HEIGHT;
    if (sdl_init(&win_width, &win_height, true) < 0) {
        FATAL("sdl_init %dx%d failed\n", win_width, win_height);
    }
    INFO("win_width=%d win_height=%d\n", win_width, win_height);

    sdl_pane_manager(
        NULL,           // context
        NULL,           // called prior to pane handlers
        NULL,           // called after pane handlers
        1000000,        // 0=continuous, -1=never, else us 
        2,              // number of pane handler varargs that follow
        sky_pane_hndlr, NULL, PH_SKY_X, PH_SKY_Y, PH_SKY_W, PH_SKY_H, PANE_BORDER_STYLE_MINIMAL,
        sky_ctl_pane_hndlr, NULL, PH_SKY_CTL_X, PH_SKY_CTL_Y, PH_SKY_CTL_W, PH_SKY_CTL_H, PANE_BORDER_STYLE_MINIMAL
                        );
}

// -----------------  UTILS  ----------------------------------------------
