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
#include <signal.h>

#include <sys/stat.h>
#include <sys/socket.h>
#include <readline/readline.h>
#include <readline/history.h>

#include "util_sdl.h"
#include "util_sdl_predefined_panes.h"
#include "util_misc.h"

//
// defines
//

#define MB 0x100000

#define TELE_CTLR_PORT 9000

#define MSG_ID_RESET     1
#define MSG_ID_CAL       2
#define MSG_ID_ENABLE    3
#define MSG_ID_DISABLE   4
#define MSG_ID_HEARTBEAT 5
#define MSG_ID_CONNECTED 6

#define MSG_ID_STR(x) \
   ((x) == MSG_ID_RESET     ? "MSG_ID_RESET"     : \
    (x) == MSG_ID_CAL       ? "MSG_ID_CAL"       : \
    (x) == MSG_ID_ENABLE    ? "MSG_ID_ENABLE"    : \
    (x) == MSG_ID_DISABLE   ? "MSG_ID_DISABLE"   : \
    (x) == MSG_ID_HEARTBEAT ? "MSG_ID_HEARTBEAT" : \
    (x) == MSG_ID_CONNECTED ? "MSG_ID_CONNECTED" : \
                              "????")

//
// typedefs
//

typedef struct {
    long long id;
    double val1;
    double val2;
    long long data_len;
    unsigned char data[0];
} msg_t;

//
// variables
//

double latitude;
double longitude;
char *ctlr_ip;

//
// prototypes
//

int sky_init(char *incl_ss_obj_str);
int sky_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event);
int sky_view_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event);

int tele_init(char *incl_ss_obj_str);
int tele_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event);

#endif
