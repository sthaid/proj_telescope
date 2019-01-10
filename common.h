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
#include <sys/socket.h>

#include "util_sdl.h"
#include "util_sdl_predefined_panes.h"
#include "util_misc.h"

//
// defines
//

#define MB 0x100000

#define TELE_CTLR_PORT 9000 // XXX tbd

#define MSG_ID_RESET     1
#define MSG_ID_CAL       2
#define MSG_ID_ENABLE    3
#define MSG_ID_DISABLE   4

#define MAX_MSG_DATA 100000


//
// typedefs
//

typedef struct {
    struct {
        int id;  // XXX
        int data_len;
    } hdr;
    union {
        struct {
            double az;
            double el;
        } cal;
        unsigned char data[MAX_MSG_DATA];
    } data;
} msg_t;

//
// variables
//

double latitude;
double longitude;
char *tele_ctlr;

//
// prototypes
//

int sky_init(char *incl_ss_obj_str);
int sky_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event);
int sky_view_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event);

int tele_init(char *incl_ss_obj_str);
int tele_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event);

#endif
