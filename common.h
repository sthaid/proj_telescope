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
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>

// help ensure portability to Raspbian, use int64_t instead of long
#define long DONT_USE_LONG

#include <util_sky.h>
#include <util_sdl.h>
#include <util_cam.h>
#include <util_wc_jpeg_decode.h>
#include <util_compress.h>
#include <util_misc.h>

//
// defines
//

#define MB 0x100000

#define TELE_CTLR_PORT 9000

#define MAX_MOTOR 2

#define MSGID_CONNECTED             1
#define MSGID_HEARTBEAT             2
#define MSGID_OPEN_ALL              3
#define MSGID_CLOSE_ALL             4
#define MSGID_STOP_ALL              5
#define MSGID_ADV_POS_SINGLE        6
#define MSGID_SET_POS_ALL           7
#define MSGID_MOTOR_STATUS          8
#define MSGID_SHUTDN_CTLR           9
#define MSGID_CAM_IMG               10
#define MSGID_CAM_IMG_ACK_RECEIPT   11
#define MSGID_CAM_RESET_REQ         12
#define MSGID_CAM_CTRLS_GET_ALL     13
#define MSGID_CAM_CTRLS_INCR_DECR   14
#define MSGID_CAM_CTRLS_GET         15
#define MSGID_CAM_CTRLS_RESET       16
#define MSGID_CAM_CTRLS_SET         17

#define MSGID_STR(x) \
   ((x) == MSGID_CONNECTED            ? "MSGID_CONNECTED"             : \
    (x) == MSGID_HEARTBEAT            ? "MSGID_HEARTBEAT"             : \
    (x) == MSGID_OPEN_ALL             ? "MSGID_OPEN_ALL"              : \
    (x) == MSGID_CLOSE_ALL            ? "MSGID_CLOSE_ALL"             : \
    (x) == MSGID_STOP_ALL             ? "MSGID_STOP_ALL"              : \
    (x) == MSGID_ADV_POS_SINGLE       ? "MSGID_ADV_POS_SINGLE"        : \
    (x) == MSGID_SET_POS_ALL          ? "MSGID_SET_POS_ALL"           : \
    (x) == MSGID_MOTOR_STATUS         ? "MSGID_MOTOR_STATUS"          : \
    (x) == MSGID_SHUTDN_CTLR          ? "MSGID_SHUTDN_CTLR"           : \
    (x) == MSGID_CAM_IMG              ? "MSGID_CAM_IMG"               : \
    (x) == MSGID_CAM_IMG_ACK_RECEIPT  ? "MSGID_CAM_IMG_ACK_RECEIPT"   : \
    (x) == MSGID_CAM_RESET_REQ        ? "MSGID_CAM_RESET_REQ"         : \
    (x) == MSGID_CAM_CTRLS_GET_ALL    ? "MSGID_CAM_CTRLS_GET_ALL"     : \
    (x) == MSGID_CAM_CTRLS_INCR_DECR  ? "MSGID_CAM_CTRLS_INCR_DECR"   : \
    (x) == MSGID_CAM_CTRLS_GET        ? "MSGID_CAM_CTRLS_GET"         : \
    (x) == MSGID_CAM_CTRLS_RESET      ? "MSGID_CAM_CTRLS_RESET"       : \
    (x) == MSGID_CAM_CTRLS_SET        ? "MSGID_CAM_CTRLS_SET"         : \
                                        "????")

#define COMPRESSION_NONE      1
#define COMPRESSION_JPEG_YUY2 2
#define COMPRESSION_LZO       3

#define PIXEL_FMT_YUY2 1
#define PIXEL_FMT_IYUV 2

//
// typedefs
//

typedef struct {
    int id;
    int data_len;
    unsigned char data[0];
} msg_t;

// XXX rename MSGID
typedef struct {
    struct motor_status_s {
        int    opened;
        int    energized;
        int    vin_voltage_mv;
        int    curr_pos_mstep;
        int    tgt_pos_mstep;
        int    spare1;
        double curr_vel_mstep_per_sec;
        char   operation_state_str[32];
        char   error_status_str[32];
    } motor[MAX_MOTOR];
} msg_motor_status_data_t;

typedef struct {
    int h;
    int mstep;
    int max_mstep;
} msg_adv_pos_single_data_t;

typedef struct {
    int mstep[MAX_MOTOR];
} msg_set_pos_all_data_t;

typedef struct {
    int pixel_fmt;
    int compression;
    int width;
    int height;
    int img_id;
    unsigned char data[0];
} msg_cam_img_data_t;

typedef struct {
    int img_id;
} msg_cam_img_ack_receipt_t;

typedef struct {
    cam_query_ctrls_t qc;
} msg_cam_ctrls_get_all_t;

typedef struct {
    int cid;
    bool incr_flag;
} msg_cam_ctrls_incr_decr_t;

typedef struct {
    int cid;
    int value;
} msg_cam_ctrls_get_t;

typedef struct {
    int cid;
    int value;
} msg_cam_ctrls_set_t;

//
// environment variables
//

double az_cal_pos;
double el_cal_pos;
char *ctlr_ip;
double az_tele_leg_1;
double min_tele_angle_relative_leg_1;
double max_tele_angle_relative_leg_1;

//
// prototypes
//

int sky_init(char *incl_ss_obj_str);
int sky_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event);
int sky_view_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event);
void sky_get_tgt_azel(double * tgt_az, double * tgt_el);

int tele_init(void);
int tele_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event);
int tele_cam_info_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event);
int tele_motor_info_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event);

#endif
