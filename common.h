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

#define MAX_MOTOR 2

#define MSGID_CONNECTED       1
#define MSGID_HEARTBEAT       2
#define MSGID_OPEN_ALL        3
#define MSGID_CLOSE_ALL       4
#define MSGID_STOP_ALL        5
#define MSGID_ADV_POS_SINGLE  6
#define MSGID_SET_POS_ALL     7
#define MSGID_STATUS          8

#define MSGID_STR(x) \
   ((x) == MSGID_CONNECTED      ? "MSGID_CONNECTED"      : \
    (x) == MSGID_HEARTBEAT      ? "MSGID_HEARTBEAT"      : \
    (x) == MSGID_OPEN_ALL       ? "MSGID_OPEN_ALL"       : \
    (x) == MSGID_CLOSE_ALL      ? "MSGID_CLOSE_ALL"      : \
    (x) == MSGID_STOP_ALL       ? "MSGID_STOP_ALL"       : \
    (x) == MSGID_ADV_POS_SINGLE ? "MSGID_ADV_POS_SINGLE" : \
    (x) == MSGID_SET_POS_ALL    ? "MSGID_SET_POS_ALL"    : \
    (x) == MSGID_STATUS         ? "MSGID_STATUS"           \
                                : "????")

//
// typedefs
//

typedef struct {
    long long id;
    long long datalen;
    unsigned char data[0];
} msg_t;

typedef struct {
    struct motor_status_s {
        long long opened;
        long long energized;
        long long vin_voltage_mv;
        long long curr_pos_mstep;
        long long tgt_pos_mstep;
        double    curr_vel_mstep_per_sec;
        long long spare1;
        long long spare2;
        char      operation_state_str[32];
        char      error_status_str[32];
    } motor[MAX_MOTOR];
} msg_status_data_t;

typedef struct {
    long long h;
    long long mstep;
    long long max_mstep;
} msg_adv_pos_single_data_t;

typedef struct {
    long long mstep[MAX_MOTOR];
} msg_set_pos_all_data_t;

//
// variables
//

double latitude;
double longitude;
double az_cal_pos;
double el_cal_pos;
char *ctlr_ip;

//
// prototypes
//

int sky_init(char *incl_ss_obj_str);
int sky_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event);
int sky_view_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event);
void sky_get_tgt_azel(double * tgt_az, double * tgt_el);

int tele_init(char *incl_ss_obj_str);
int tele_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event);

#endif
