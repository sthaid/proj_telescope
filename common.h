#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stddef.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <limits.h>
#include <assert.h>
#include <pthread.h>
#include <math.h>
#include <sys/stat.h>
#include <readline/readline.h>
#include <readline/history.h>

#include "util_sdl.h"
#include "util_sdl_predefined_panes.h"
#include "util_misc.h"

//
// defines
//

#define MB 0x100000

//
// typedefs
//

//
// variables
//

double latitude;
double longitude;

//
// prototypes
//

int sky_init(char *incl_ss_obj_str);
int sky_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event);
int sky_ctl_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event);
int sky_view_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event);

#endif
