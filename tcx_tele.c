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
 
#define AZDEG_TO_MSTEP(deg)    (rint((deg) * ((200. * 32. * 6.) / 360.)))
#define MSTEP_TO_AZDEG(mstep)  ((mstep) * (360. / (200. * 32. * 6.)))
#define AZMSTEP_360_DEG        (200 * 32 * 6)
#define AZMSTEP_180_DEG        (AZMSTEP_360_DEG / 2)
#define AZMSTEP_1_DEG          ((int)(AZMSTEP_360_DEG / 360. + .5))

#define ELDEG_TO_MSTEP(deg)    (rint((deg) * ((200. * 32. * 6.) / 360.)))
#define MSTEP_TO_ELDEG(mstep)  ((mstep) * (360. / (200. * 32. * 6.)))
#define ELMSTEP_360_DEG        (200 * 32 * 6)
#define ELMSTEP_180_DEG        (ELMSTEP_360_DEG / 2)
#define ELMSTEP_1_DEG          ((int)(ELMSTEP_360_DEG / 360. + .5))

#define SDL_EVENT_TELE_MOTORS_CLOSE   (SDL_EVENT_USER_DEFINED + 0)
#define SDL_EVENT_TELE_MOTORS_OPEN    (SDL_EVENT_USER_DEFINED + 1)
#define SDL_EVENT_TELE_UN_CALIBRATE   (SDL_EVENT_USER_DEFINED + 2)
#define SDL_EVENT_TELE_CALIBRATE      (SDL_EVENT_USER_DEFINED + 3)
#define SDL_EVENT_TELE_TRK_DISABLE    (SDL_EVENT_USER_DEFINED + 4)
#define SDL_EVENT_TELE_TRK_ENABLE     (SDL_EVENT_USER_DEFINED + 5)
#define SDL_EVENT_TELE_SHUTDN_CTLR    (SDL_EVENT_USER_DEFINED + 6)
#define SDL_EVENT_TELE_MOUSE_MOTION   (SDL_EVENT_USER_DEFINED + 7)
#define SDL_EVENT_TELE_MOUSE_WHEEL    (SDL_EVENT_USER_DEFINED + 8)

#define EVENT_ID_STR(x) \
   ((x) == SDL_EVENT_TELE_MOTORS_CLOSE     ? "MOTORS_CLOSE"          : \
    (x) == SDL_EVENT_TELE_MOTORS_OPEN      ? "MOTORS_OPEN"           : \
    (x) == SDL_EVENT_TELE_UN_CALIBRATE     ? "UN_CALIBRATE"          : \
    (x) == SDL_EVENT_TELE_CALIBRATE        ? "CALIBRATE"             : \
    (x) == SDL_EVENT_TELE_TRK_DISABLE      ? "TRK_DISABLE"           : \
    (x) == SDL_EVENT_TELE_TRK_ENABLE       ? "TRK_ENABLE"            : \
    (x) == SDL_EVENT_TELE_SHUTDN_CTLR      ? "SHUTDN_CTLR"           : \
    (x) == SDL_EVENT_TELE_MOUSE_MOTION     ? "MOUSE_MOTION"          : \
    (x) == SDL_EVENT_TELE_MOUSE_WHEEL      ? "MOUSE_WHEEL"           : \
    (x) == SDL_EVENT_KEY_LEFT_ARROW        ? "KEY_LEFT_ARROW"        : \
    (x) == SDL_EVENT_KEY_RIGHT_ARROW       ? "KEY_RIGHT_ARROW"       : \
    (x) == SDL_EVENT_KEY_UP_ARROW          ? "KEY_UP_ARROW"          : \
    (x) == SDL_EVENT_KEY_DOWN_ARROW        ? "KEY_DOWN_ARROW"        : \
    (x) == SDL_EVENT_KEY_SHIFT_LEFT_ARROW  ? "KEY_SHIFT_LEFT_ARROW"  : \
    (x) == SDL_EVENT_KEY_SHIFT_RIGHT_ARROW ? "KEY_SHIFT_RIGHT_ARROW" : \
    (x) == SDL_EVENT_KEY_SHIFT_UP_ARROW    ? "KEY_SHIFT_UP_ARROW"    : \
    (x) == SDL_EVENT_KEY_SHIFT_DOWN_ARROW  ? "KEY_SHIFT_DOWN_ARROW"  : \
                                             "????")

#define MOTORS_CLOSED 0
#define MOTORS_OPEN   1
#define MOTORS_ERROR  2

#define MOTORS_STR(x) \
   ((x) == MOTORS_CLOSED ? "CLOSED" : \
    (x) == MOTORS_OPEN   ? "OPEN"   : \
    (x) == MOTORS_ERROR  ? "ERROR"  : \
                           "????")

#define CTLR_MOTOR_STATUS_VALID (microsec_timer() - ctlr_motor_status_us <= 5000000)
#define CAM_IMG_VALID           (microsec_timer() - cam_img.recv_time_us[0] < 5000000)
#define CAM_CTRLS_VALID         (microsec_timer() - vars->cam_ctrls_time_us < 5000000)

#define SDLPR(fmt, args...) \
    do { \
        sdl_render_printf(pane, COL2X(sdlpr_col,fontsz), ROW2Y(sdlpr_row,fontsz), fontsz, WHITE, BLACK, fmt, ## args); \
        sdlpr_row++; \
    } while (0)

#define MAX_CAM_IMG_RECV_TIME_US 20

//
// typedefs
//

typedef struct {
    unsigned char * pixels;
    int width;
    int height;
    int pixel_fmt;
    uint64_t recv_time_us[MAX_CAM_IMG_RECV_TIME_US];
    int recv_count;
} cam_img_t;

//
// variables
//

// comm to tele ctlr vars
static int      sfd = -1;
static bool     connected;
static uint64_t connection_established_time_us;

// telescope motor status vars
static msg_motor_status_data_t ctlr_motor_status;
static uint64_t          ctlr_motor_status_us;

// telsecope control vars
static int    motors;
static bool   calibrated;
static bool   tracking_enabled;

static int    cal_az0_mstep;
static int    cal_el0_mstep;

static double tgt_az, tgt_el;     // az range -180 to +180
static bool   tgt_azel_valid;

static double act_az, act_el;     // az range -180 to +180
static bool   act_azel_available;
static bool   act_azel_valid;

static int    adj_az_mstep, adj_el_mstep;

static double min_valid_az, max_valid_az;  // az range -180 to +180

static pthread_mutex_t tele_ctrl_mutex = PTHREAD_MUTEX_INITIALIZER;

// webcam image vars
static cam_img_t       cam_img;
static pthread_mutex_t cam_img_mutex = PTHREAD_MUTEX_INITIALIZER;

// webcam ctrl vars
static char                cam_ctrls_buff[10000];
static cam_query_ctrls_t * cam_ctrls = (void*)cam_ctrls_buff; 
static int                 cam_ctrls_len;
static uint64_t            cam_ctrls_time_us;
static pthread_mutex_t     cam_ctrls_mutex = PTHREAD_MUTEX_INITIALIZER;

//
// prototypes
//

static void * comm_thread(void * cx);
static void * comm_heartbeat_thread(void * cx);
static int comm_process_recvd_msg(msg_t * msg);
static void comm_send_msg(msg_t * msg);
static void comm_set_sock_opts(void);
static void comm_verify_sock_opts(void);

static void * tele_ctrl_thread(void * cx);
static void tele_ctrl_process_cmd(int event_id);
static void tele_ctrl_get_status(char *str1, char *str2, char *str3, char *str4, char *str5, char *str6);
static bool tele_ctrl_is_azel_valid(double az, double el);

static void sanitize_pan_zoom(void);

// 
// inline procedures
//

// returns az in range -180 to 179.99999
static inline double sanitize_az(double az) 
{
    if (az >= 180) {
        while (az >= 180) az -= 360;
    } else {
        while (az < -180) az += 360;
    }
    return az;
}

// -----------------  TELE INIT  ------------------------------------------

int tele_init(void)
{
    pthread_t thread_id;

    // this performs sanity checks for minilzo
    compress_init();

    // determine min_valid_az and max_valid_az, 
    // used by tele_ctrl_is_azel_valid()
    min_valid_az = az_tele_leg_1 + min_tele_angle_relative_leg_1;
    max_valid_az = az_tele_leg_1 + max_tele_angle_relative_leg_1;
    min_valid_az = sanitize_az(min_valid_az);
    max_valid_az = sanitize_az(max_valid_az);
    INFO("min_valid_az %0.2lf   max_valid_az %0.2lf\n", min_valid_az, max_valid_az);

    // create threads
    pthread_create(&thread_id, NULL, comm_thread, NULL);
    pthread_create(&thread_id, NULL, comm_heartbeat_thread, NULL);
    pthread_create(&thread_id, NULL, tele_ctrl_thread, NULL);

    // return success
    return 0;
}

// -----------------  COMM TO TELE CTLR  ----------------------------------

static void * comm_thread(void * cx)
{
    static char msg_buffer[10000000];

    int rc, sfd_temp, len;
    struct sockaddr_in addr;
    msg_t * msg = (msg_t*)msg_buffer;

reconnect:
    // connect to telescope ctlr  (raspberry pi)
    sfd = socket(AF_INET, SOCK_STREAM, 0);
    if (getsockaddr(ctlr_ip, TELE_CTLR_PORT, &addr) < 0) {
        FATAL("failed to get address of %s\n", ctlr_ip);
    }
    do {
        rc = connect(sfd, (struct sockaddr *)&addr, sizeof(addr));
        if (rc == -1) {
            DEBUG("failed connect to %s, %s\n", ctlr_ip, strerror(errno));
            sleep(1);
        }
    } while (rc == -1);

    // set and verify socket options
    comm_set_sock_opts();
    comm_verify_sock_opts();

    // send connected msg
    memset(msg,0,sizeof(msg_t));
    msg->id = MSGID_CONNECTED;
    comm_send_msg(msg);

    // on new connection should first recv MSGID_CONNECTED
    len = do_recv(sfd, msg, sizeof(msg_t));
    if (len != sizeof(msg_t)) {
        ERROR("recvd initial msg with invalid len %d, %s\n", len, strerror(errno));
        goto lost_connection;
    }
    if (msg->id != MSGID_CONNECTED) {
        ERROR("recvd invalid initial msg, id=%d\n", msg->id);
        goto lost_connection;
    }
    INFO("established connection to telescope\n");
    connected = true;
    connection_established_time_us = microsec_timer();

    // receive msgs from ctlr_ip, and
    // process them
    while (true) {
        // recv msg
        len = do_recv(sfd, msg, sizeof(msg_t));
        if (len != sizeof(msg_t)) {
            ERROR("recvd msg with invalid len %d, %s\n", len, strerror(errno));
            break;
        }
        len = do_recv(sfd, msg->data, msg->data_len);
        if (len != msg->data_len) {
            ERROR("recvd msg data with invalid len %d, %s\n", len, strerror(errno));
            break;
        }

        // process the recvd msg
        rc = comm_process_recvd_msg(msg);
        if (rc != 0) {
            break;
        }
    }

lost_connection:
    // lost connection; reconnect
    connected = false;
    connection_established_time_us = 0;
    sfd_temp = sfd;
    sfd = -1;
    close(sfd_temp);
    ERROR("lost connection to telescope, attempting to reconnect\n");
    goto reconnect;

    return NULL;
}

static int comm_process_recvd_msg(msg_t * msg)
{
    #define CHECK_DATALEN(explen) \
        do { \
            if (msg->data_len != explen) { \
                ERROR("incorrect data_len %d in %s\n", msg->data_len, MSGID_STR(msg->id)); \
                break; \
            } \
        } while (0)

    DEBUG("received %s data_len %d\n", MSGID_STR(msg->id), msg->data_len);

    switch (msg->id) {
    case MSGID_MOTOR_STATUS: {
        CHECK_DATALEN(sizeof(msg_motor_status_data_t));
        ctlr_motor_status = *(msg_motor_status_data_t *)msg->data;
        ctlr_motor_status_us = microsec_timer();
        break; }
    case MSGID_CAM_IMG: {
        msg_cam_img_data_t * msg_data = (void*)msg->data;
        void               * data = msg_data->data;
        int                  data_len = msg->data_len - sizeof(msg_cam_img_data_t);
        unsigned char      * pixels = NULL;
        size_t               pixels_len = 0;
        int                  expected_pixels_len;
        bool                 error;

        char msg_cam_img_ack_buffer[100];
        msg_t *msg_cam_img_ack = (void*)msg_cam_img_ack_buffer;
        msg_cam_img_ack_receipt_t * msg_cam_img_ack_receipt = (void*)msg_cam_img_ack->data;

        // send message back to ctlr acknowleging receipt of the cam img
        msg_cam_img_ack->id = MSGID_CAM_IMG_ACK_RECEIPT;
        msg_cam_img_ack->data_len = sizeof(msg_cam_img_ack_receipt_t);
        msg_cam_img_ack_receipt->img_id = msg_data->img_id;
        comm_send_msg(msg_cam_img_ack);

        // get pixels based on type of compression
        switch (msg_data->compression) {
        case COMPRESSION_NONE:
            pixels = malloc(data_len);
            memcpy(pixels, data, data_len);
            pixels_len = data_len;
            error = false;
            break;
        case COMPRESSION_JPEG_YUY2: {
            int rc;
            unsigned int w, h;
            rc = jpeg_decode(0, JPEG_DECODE_MODE_YUY2, data, data_len, &pixels, &w, &h);
            if (rc != 0) {
                ERROR("jpeg_decode rc %d\n", rc);
                error = true;
                break;
            }
            if (w != msg_data->width || h != msg_data->height) {
                ERROR("jpeg_decode w=%d h=%d, expected w=%d h=%d\n", 
                      w, h, msg_data->width, msg_data->height);
                error = true;
                break;
            }
            pixels_len = 2 * w * h;
            error = false;
            break; }
        case COMPRESSION_LZO:
            pixels = malloc(1000000);
            pixels_len = 1000000;
            error = (decompress(data, data_len, pixels, &pixels_len) != 0);
            if (error) {
                ERROR("decompress failed, data_len=%d pixels_len=%zd\n", data_len, pixels_len);
            }
            break;
        default:
            error = true;
        }

        // if an error occurred then get out
        if (error) {
            free(pixels);
            break;
        }

        // check pixels_len, if invalid then get out
        expected_pixels_len = 
            msg_data->pixel_fmt == PIXEL_FMT_YUY2 ? msg_data->width * msg_data->height * 2 :
            msg_data->pixel_fmt == PIXEL_FMT_IYUV ? msg_data->width * msg_data->height * 3 / 2 :
                                                    -1;
        if (pixels_len != expected_pixels_len) {
            ERROR("pixels_len %zd is not expected %d\n", pixels_len, expected_pixels_len);
            free(pixels);
            break;
        }

        // make the pixels available to tele_pane_hndlr
        pthread_mutex_lock(&cam_img_mutex);
        free(cam_img.pixels);
        cam_img.pixels    = pixels;
        cam_img.width     = msg_data->width;
        cam_img.height    = msg_data->height;
        cam_img.pixel_fmt = msg_data->pixel_fmt;
        memmove(&cam_img.recv_time_us[1], &cam_img.recv_time_us[0], (MAX_CAM_IMG_RECV_TIME_US-1)*sizeof(uint64_t));
        cam_img.recv_time_us[0] = microsec_timer();
        cam_img.recv_count++;
        pthread_mutex_unlock(&cam_img_mutex);
        break; }
    case MSGID_HEARTBEAT:
        CHECK_DATALEN(0);
        break;
    case MSGID_CAM_CTRLS_GET_ALL:
        pthread_mutex_lock(&cam_ctrls_mutex);
        memcpy(cam_ctrls, msg->data, msg->data_len);
        cam_ctrls_len = msg->data_len;
        cam_ctrls_time_us = microsec_timer();
        pthread_mutex_unlock(&cam_ctrls_mutex);
        break;
    case MSGID_CAM_CTRLS_GET: {
        msg_cam_ctrls_get_t *d = (void*)msg->data;
        int i;
        for (i = 0; i < cam_ctrls->max_cam_ctrl; i++) {
            struct cam_ctrl_s *x = &cam_ctrls->cam_ctrl[i];
            if (x->cid == d->cid) {
                x->current_value = d->value;
                break;
            }
        }
        if (i == cam_ctrls->max_cam_ctrl) {
            ERROR("rcvd MSGID_CAM_CTRLS_GET: cid 0x%x not found in cam_ctrls\n", d->cid);
        }
        break; }
    default:
        ERROR("invalid msgid %d\n", msg->id);
        return -1;
    }

    return 0;
}

static void comm_send_msg(msg_t * msg)
{
    int len;
    static pthread_mutex_t send_mutex = PTHREAD_MUTEX_INITIALIZER;

    if (!connected && msg->id != MSGID_CONNECTED) {
        return;
    }

    DEBUG("sending %s\n", MSGID_STR(msg->id));

    pthread_mutex_lock(&send_mutex);
    len = do_send(sfd, msg, sizeof(msg_t)+msg->data_len);
    pthread_mutex_unlock(&send_mutex);

    if (len != sizeof(msg_t)+msg->data_len) {
        ERROR("send failed, len=%d, %s\n", len, strerror(errno));
    }
}

static void * comm_heartbeat_thread(void * cx)
{
    msg_t msg;

    memset(&msg,0,sizeof(msg_t));
    msg.id = MSGID_HEARTBEAT;

    while (true) {
        if (connected) {
            comm_send_msg(&msg);
        }
        usleep(200000);
    }

    return NULL;
}

static void comm_set_sock_opts(void)
{
    int rc, optval;

    if (sfd == -1) {
        FATAL("BUG: sfd is -1\n");
    }

    // set send-buffer size 
    optval = MB;
    rc = setsockopt(sfd, SOL_SOCKET, SO_SNDBUF, &optval, sizeof(optval));
    if (rc == -1) {
        FATAL("setsockopt SO_SNDBUF, %s", strerror(errno));
    }

    // set rcv-buffer size
    optval = MB;
    rc = setsockopt(sfd, SOL_SOCKET, SO_RCVBUF, &optval, sizeof(optval));
    if (rc == -1) {
        FATAL("setsockopt SO_RCVBUF, %s", strerror(errno));
    }

    // enable TCP_NODELAY
    optval = 1;
    rc = setsockopt(sfd, IPPROTO_TCP, TCP_NODELAY, &optval, sizeof(optval));
    if (rc == -1) {
        FATAL("setsockopt TCP_NODELAY, %s", strerror(errno));
    }
}

static void comm_verify_sock_opts(void)
{
    int rc, optval;
    socklen_t optlen;

    if (sfd == -1) {
        FATAL("BUG: sfd is -1\n");
    }

    // verify send-buffer size
    optval = 0;
    optlen = sizeof(optval);
    rc = getsockopt(sfd, SOL_SOCKET, SO_SNDBUF, &optval, &optlen);       
    if (rc == -1) {
        FATAL("getsockopt SO_SNDBUF, %s", strerror(errno));
    }
    if (optval != 2*MB) {
        INFO("add to /etc/sysctl.conf:\n");
        INFO("   net.core.wmem_max = 2000000\n");
        INFO("   net.core.rmem_max = 2000000\n");
        FATAL("getsockopt SO_SNDBUF, optval=%d\n", optval);
    }

    // verify recv-buffer size
    optval = 0;
    optlen = sizeof(optval);
    rc = getsockopt(sfd, SOL_SOCKET, SO_RCVBUF, &optval, &optlen);       
    if (rc == -1) {
        FATAL("getsockopt SO_RCVBUF, %s", strerror(errno));
    }
    if (optval != 2*MB) {
        INFO("add to /etc/sysctl.conf:\n");
        INFO("   net.core.wmem_max = 2000000\n");
        INFO("   net.core.rmem_max = 2000000\n");
        FATAL("getsockopt SO_RCVBUF, optval=%d\n", optval);
    }

    // verify TCP_NODELAY
    optval = 0;
    optlen = sizeof(optval);
    rc = getsockopt(sfd, IPPROTO_TCP, TCP_NODELAY, &optval, &optlen);       
    if (rc == -1) {
        FATAL("getsockopt TCP_NODELAY, %s", strerror(errno));
    }
    if (optval != 1) {
        FATAL("getsockopt TCP_NODELAY, optval=%d\n", optval);
    }
}

// -----------------  TELE CONTROL  ---------------------------------------

static void * tele_ctrl_thread(void * cx)
{
    uint64_t time_us, time_last_set_pos_us=0;

    bool connected_last = false;
    int  motors_last = MOTORS_CLOSED;
    bool calibrated_last = false;
    bool tracking_enabled_last = false;
    bool tgt_azel_valid_last = false;
    bool act_azel_valid_last = false;

    while (true) {
        // delay 50 ms
        usleep(50000);

        // lock mutex
        pthread_mutex_lock(&tele_ctrl_mutex);

        // determine motor status from ctlr_motor_status
        if (!connected) {
            if (motors != MOTORS_ERROR) {
                INFO("setting MOTORS_ERROR because not connected\n");
            }
            motors = MOTORS_ERROR;
        } else if (!CTLR_MOTOR_STATUS_VALID) {
            if (motors != MOTORS_ERROR) {
                INFO("setting MOTORS_ERROR because because not CTLR_MOTOR_STATUS_VALID\n");
            }
            motors = MOTORS_ERROR;
        } else if (ctlr_motor_status.motor[0].opened == 0 && ctlr_motor_status.motor[1].opened == 0) {
            if (motors != MOTORS_CLOSED) {
                INFO("setting MOTORS_CLOSED because because one or both motors are not open (%d %d)\n",
                    ctlr_motor_status.motor[0].opened , ctlr_motor_status.motor[1].opened);
            }
            motors = MOTORS_CLOSED;
        } else if (!(ctlr_motor_status.motor[0].opened == 1 &&
                     strcmp(ctlr_motor_status.motor[0].operation_state_str, "NORMAL") == 0 &&
                     strcmp(ctlr_motor_status.motor[0].error_status_str, "NO_ERR") == 0 &&
                     ctlr_motor_status.motor[1].opened == 1 &&
                     strcmp(ctlr_motor_status.motor[1].operation_state_str, "NORMAL") == 0 &&
                     strcmp(ctlr_motor_status.motor[1].error_status_str, "NO_ERR") == 0))
        {
            // XXX may need to lock with update to ctlr_motor_status
            if (motors != MOTORS_ERROR) {
                INFO("setting MOTORS_ERROR because motor0=%d,%s,%s motor1=%d,%s,%s\n",
                     ctlr_motor_status.motor[0].opened,
                     ctlr_motor_status.motor[0].operation_state_str,
                     ctlr_motor_status.motor[0].error_status_str,
                     ctlr_motor_status.motor[1].opened,
                     ctlr_motor_status.motor[1].operation_state_str,
                     ctlr_motor_status.motor[1].error_status_str);
            }
            motors = MOTORS_ERROR;
        } else {
            if (motors != MOTORS_OPEN) {
                INFO("setting MOTORS_OPEN because all is well\n");
            }
            motors = MOTORS_OPEN;
        }

        // debug print if the receipt of motor_status is delayed 
        if (connected && ctlr_motor_status_us) {
            uint64_t motor_status_age_us, ms_us;
            ms_us = ctlr_motor_status_us;
            __sync_synchronize();
            motor_status_age_us = microsec_timer() - ms_us;
            if (motor_status_age_us > 1000000) {
                WARN("motor_status delayed receipt, age = %ld us\n", motor_status_age_us);
            }
        }

        // if telescope is not fully ready then ensure the 
        // calibrated and tracking_enabled flags are clear
        // and zero the az/el adjustment values
        if (!connected || motors != MOTORS_OPEN || !calibrated) {
            calibrated = false;
            tracking_enabled = false;
            adj_az_mstep = adj_el_mstep = 0;
        }

        // XXX if connected and motorsopen and !calibrated and 
        //             recently rcvd  tele_cal msg avail
        //     then accept the tele_cal and clear it
        // XXX LATER

        // get tgt_az/el by calling sky routine, and
        // determine if the target azel is valid (can the telescope mechanism point there)
        sky_get_tgt_azel(&tgt_az, &tgt_el);
        tgt_az = sanitize_az(tgt_az);
        tgt_azel_valid = tele_ctrl_is_azel_valid(tgt_az, tgt_el);

        // determine act_az/el from ctrl_motor_status shaft position
        if (calibrated) {
            int curr_az_mstep = ctlr_motor_status.motor[0].curr_pos_mstep;
            int curr_el_mstep = ctlr_motor_status.motor[1].curr_pos_mstep;
            act_az = MSTEP_TO_AZDEG(curr_az_mstep - cal_az0_mstep - adj_az_mstep);
            act_az = sanitize_az(act_az);
            act_el = MSTEP_TO_ELDEG(curr_el_mstep - cal_el0_mstep - adj_el_mstep);
            act_azel_available = true;
            act_azel_valid = tele_ctrl_is_azel_valid(act_az, act_el);
        } else {
            act_azel_available = false;
            act_azel_valid = false;
        }

        // if the target or actual azel is invalid then disable tracking
        if (!tgt_azel_valid || !act_azel_valid) {
            tracking_enabled = false;
        }

        // if tracking is enabled then set telescope position to tgt_az,tgt_el;
        // do this once per second
        if (tracking_enabled &&
            (time_us = microsec_timer()) >= time_last_set_pos_us + 1000000)
        {
            char msg_buffer[sizeof(msg_t)+sizeof(msg_set_pos_all_data_t)];
            msg_t *msg = (void*)msg_buffer;
            msg_set_pos_all_data_t *msg_set_pos_all_data = (void*)(msg+1);
            int az_mstep, el_mstep;

            // determine az/el microstep motor positions based on caliabration
            az_mstep = cal_az0_mstep + AZDEG_TO_MSTEP(tgt_az) + adj_az_mstep;
            el_mstep = cal_el0_mstep + ELDEG_TO_MSTEP(tgt_el) + adj_el_mstep;

            // format and send MSGID_SET_POS_ALL to the telescope ctrlr
            msg->id = MSGID_SET_POS_ALL;
            msg->data_len = sizeof(msg_set_pos_all_data_t);
            msg_set_pos_all_data->mstep[0] = az_mstep;
            msg_set_pos_all_data->mstep[1] = el_mstep;
            comm_send_msg(msg);

            // keep track of time MSGID_SET_POS_ALL was sent so that 
            // we limit sending this message to once per sec
            time_last_set_pos_us = time_us;
        }

        // if tracking has just been disabled or 
        //    target azel has just become invalid 
        //    actual azel has just become invalid 
        // then stop the motors
        if ((tracking_enabled_last && !tracking_enabled) ||
            (tgt_azel_valid_last && !tgt_azel_valid) ||
            (act_azel_valid_last && !act_azel_valid))
        {
            msg_t msg = { MSGID_STOP_ALL, 0 };
            INFO("stopping motors\n");
            comm_send_msg(&msg);
        }

        // debug print changes
        if (connected_last != connected ||
            motors_last != motors ||
            calibrated_last != calibrated ||
            tracking_enabled_last != tracking_enabled ||
            tgt_azel_valid_last != tgt_azel_valid ||
            act_azel_valid_last != act_azel_valid)
        {
            INFO("state changed to: connected=%d motors=%s calibrated=%d tracking_enabled=%d "
                                   "tgt_azel_valid=%d act_azel_valid=%d\n",
                 connected, 
                 MOTORS_STR(motors), 
                 calibrated, 
                 tracking_enabled,
                 tgt_azel_valid,
                 act_azel_valid);
        }

        // save last values
        connected_last = connected;
        motors_last = motors;
        calibrated_last = calibrated;
        tracking_enabled_last = tracking_enabled;
        tgt_azel_valid_last = tgt_azel_valid;
        act_azel_valid_last = act_azel_valid;

        // unlock mutex
        pthread_mutex_unlock(&tele_ctrl_mutex);
    }

    return NULL;
}

static void tele_ctrl_process_cmd(int event_id)
{
    // lock mutex
    pthread_mutex_lock(&tele_ctrl_mutex);

    DEBUG("processing event_id %s (0x%x)\n", EVENT_ID_STR(event_id), event_id);

    switch (event_id) {
    case SDL_EVENT_TELE_MOTORS_CLOSE:
        if (connected && motors != MOTORS_CLOSED) {
            msg_t msg = { MSGID_CLOSE_ALL, 0 };
            comm_send_msg(&msg);
            calibrated = false;
            tracking_enabled = false;
        }
        break;
    case SDL_EVENT_TELE_MOTORS_OPEN:
        if (connected && motors == MOTORS_CLOSED) {
            msg_t msg = { MSGID_OPEN_ALL, 0 };
            comm_send_msg(&msg);
            calibrated = false;
            tracking_enabled = false;
        }
        break;
    case SDL_EVENT_TELE_UN_CALIBRATE:
        if (calibrated) {
            calibrated = false;
            tracking_enabled = false;
        }
        break;
    case SDL_EVENT_TELE_CALIBRATE:
        if (connected && motors == MOTORS_OPEN && !calibrated) {
            int curr_az_mstep = ctlr_motor_status.motor[0].curr_pos_mstep;
            int curr_el_mstep = ctlr_motor_status.motor[1].curr_pos_mstep;
            cal_az0_mstep = curr_az_mstep - AZDEG_TO_MSTEP(tgt_az);
            cal_el0_mstep = curr_el_mstep - ELDEG_TO_MSTEP(tgt_el);
            INFO("calibrated:    cal_mstep curr_mstep target\n");
            INFO("calibrated: AZ %9d %10d %6.2f\n", cal_az0_mstep, curr_az_mstep, tgt_az);
            INFO("calibrated: EL %9d %10d %6.2f\n", cal_el0_mstep, curr_el_mstep, tgt_el);
            calibrated = true;
            tracking_enabled = false;
        }
        break;
    case SDL_EVENT_TELE_TRK_DISABLE:
        if (calibrated) {
            tracking_enabled = false;
        }
        break;
    case SDL_EVENT_TELE_TRK_ENABLE:
        if (calibrated) {
            tracking_enabled = true;
        }
        break;
    case SDL_EVENT_TELE_SHUTDN_CTLR:
        if (connected && motors == MOTORS_CLOSED) {
            msg_t msg = { MSGID_SHUTDN_CTLR, 0 };
            comm_send_msg(&msg);
            calibrated = false;
            tracking_enabled = false;
        }
        break;
    case SDL_EVENT_KEY_UP_ARROW:
    case SDL_EVENT_KEY_DOWN_ARROW:
    case SDL_EVENT_KEY_LEFT_ARROW:
    case SDL_EVENT_KEY_RIGHT_ARROW:
    case SDL_EVENT_KEY_SHIFT_UP_ARROW:
    case SDL_EVENT_KEY_SHIFT_DOWN_ARROW:
    case SDL_EVENT_KEY_SHIFT_LEFT_ARROW:
    case SDL_EVENT_KEY_SHIFT_RIGHT_ARROW: {
        if (calibrated) {
            if (event_id == SDL_EVENT_KEY_LEFT_ARROW || event_id == SDL_EVENT_KEY_RIGHT_ARROW) {
                adj_az_mstep += (event_id == SDL_EVENT_KEY_RIGHT_ARROW ? 1 : -1);
                if (adj_az_mstep < -AZMSTEP_1_DEG) adj_az_mstep = -AZMSTEP_1_DEG;
                if (adj_az_mstep > AZMSTEP_1_DEG) adj_az_mstep = AZMSTEP_1_DEG;
            } else if (event_id == SDL_EVENT_KEY_UP_ARROW || event_id == SDL_EVENT_KEY_DOWN_ARROW) {
                adj_el_mstep += (event_id == SDL_EVENT_KEY_UP_ARROW ? 1 : -1);
                if (adj_el_mstep < -ELMSTEP_1_DEG) adj_el_mstep = -ELMSTEP_1_DEG;
                if (adj_el_mstep > ELMSTEP_1_DEG) adj_el_mstep = ELMSTEP_1_DEG;
            }
            INFO("adj azel = %d %d mstep   %f %f deg\n", 
                 adj_az_mstep, adj_el_mstep,
                 MSTEP_TO_AZDEG(adj_az_mstep), MSTEP_TO_ELDEG(adj_el_mstep));
        } else if (connected && motors == MOTORS_OPEN) {
            char msg_buffer[sizeof(msg_t)+sizeof(msg_adv_pos_single_data_t)];
            msg_t *msg = (void*)msg_buffer;
            msg_adv_pos_single_data_t *msg_adv_pos_single_data = (void*)(msg+1);
            int h = (event_id == SDL_EVENT_KEY_LEFT_ARROW || event_id == SDL_EVENT_KEY_RIGHT_ARROW ||
                     event_id == SDL_EVENT_KEY_SHIFT_LEFT_ARROW || event_id == SDL_EVENT_KEY_SHIFT_RIGHT_ARROW)
                     ? 0 : 1;
            int sign = (event_id == SDL_EVENT_KEY_UP_ARROW || event_id == SDL_EVENT_KEY_SHIFT_UP_ARROW ||
                        event_id == SDL_EVENT_KEY_RIGHT_ARROW || event_id == SDL_EVENT_KEY_SHIFT_RIGHT_ARROW) 
                        ? 1 : -1;
            bool fine = (event_id == SDL_EVENT_KEY_SHIFT_LEFT_ARROW || event_id == SDL_EVENT_KEY_SHIFT_RIGHT_ARROW ||
                         event_id == SDL_EVENT_KEY_SHIFT_UP_ARROW || event_id == SDL_EVENT_KEY_SHIFT_DOWN_ARROW);
            int mstep, max_mstep;

            if (h == 0 ) {
                max_mstep = AZDEG_TO_MSTEP(1);
                mstep = (fine ? sign : sign * AZDEG_TO_MSTEP(0.1));
            } else {
                max_mstep = ELDEG_TO_MSTEP(1);
                mstep = (fine ? sign : sign * ELDEG_TO_MSTEP(0.1));
            }

            msg->id = MSGID_ADV_POS_SINGLE;
            msg->data_len = sizeof(msg_adv_pos_single_data_t);
            msg_adv_pos_single_data->h = h;
            msg_adv_pos_single_data->mstep = mstep;
            msg_adv_pos_single_data->max_mstep = max_mstep;

            comm_send_msg(msg);
        }
        break; }
    }

    // unlock mutex
    pthread_mutex_unlock(&tele_ctrl_mutex);
}

static void tele_ctrl_get_status(char *str1, char *str2, char *str3, char *str4, char *str5, char *str6)
{
    double tgt_az2, act_az2;
    bool acquired = false;

    // preset returns to empty strings
    str1[0] = '\0';
    str2[0] = '\0';
    str3[0] = '\0';
    str4[0] = '\0';
    str5[0] = '\0';
    str6[0] = '\0';

    // lock mutex
    pthread_mutex_lock(&tele_ctrl_mutex);

    // the target and actual azimuth values returned in the strings 
    // are adjusted to range 0-360
    tgt_az2 = (tgt_az >= 0 ? tgt_az : tgt_az+360);
    act_az2 = (act_az >= 0 ? act_az : act_az+360);

    // STATUS LINE 1 - upper left
    //   DISCONNECTED
    //   MOTORS_CLOSED
    //   MOTORS_ERROR
    //   UNCALIBRATED
    //   BAD_AZEL
    //   DISABLED
    //   ACQUIRING
    //   ACQUIRED
    if (!connected) {
        sprintf(str1, "DISCON %s", ctlr_ip);
    } else if (motors == MOTORS_CLOSED) {
        sprintf(str1, "MTRS_CLOSED");
    } else if (motors == MOTORS_ERROR) {
        sprintf(str1, "MTRS_ERROR");
    } else if (!calibrated) {
        sprintf(str1, "UN_CAL");
    } else if (!tgt_azel_valid) {
        sprintf(str1, "BAD_TGT_AZEL");
    } else if (!tracking_enabled) {
        sprintf(str1, "DISABLED");
    } else {
        double az_delta = fabs(act_az2-tgt_az2);
        double el_delta = fabs(act_el-tgt_el);
        if (az_delta > 180) {
            az_delta = fabs(360-az_delta);
        }
        acquired = (az_delta <= 0.02 && el_delta <= 0.02);
        if (!acquired) {
            sprintf(str1, "ACQUIRING");
        } else {
            sprintf(str1, "ACQUIRED");
        }
    }

    // STATUS LINE 2 - upper left
    //    MOTOR xxx xxx   (when not calibrated and connected and motors are not closed
    //    TGT az el       (when calibrated)
    if (!calibrated && connected && motors != MOTORS_CLOSED) {
        char motor0_pos_str[32];
        char motor1_pos_str[32];
        if (CTLR_MOTOR_STATUS_VALID && ctlr_motor_status.motor[0].opened) {
            sprintf(motor0_pos_str, "%d", ctlr_motor_status.motor[0].curr_pos_mstep);
        } else {
            strcpy(motor0_pos_str, "-");
        }
        if (CTLR_MOTOR_STATUS_VALID && ctlr_motor_status.motor[1].opened) {
            sprintf(motor1_pos_str, "%d", ctlr_motor_status.motor[1].curr_pos_mstep);
        } else {
            strcpy(motor1_pos_str, "-");
        }
        sprintf(str2, "MOTOR %s %s", motor0_pos_str, motor1_pos_str);
    } else if (calibrated) {
        sprintf(str2, "TGT %6.2f %6.2f", tgt_az2, tgt_el);
    }

    // STATUS LINE 3 - upper left
    //    ACT az el       (when act_azel_available and (not-valid or not-acquired)
    if (act_azel_available && (!act_azel_valid || !acquired)) {
        if (act_azel_valid) {
            sprintf(str3, "ACT %6.2f %6.2f", act_az2, act_el);
        } else {
            sprintf(str3, "ACT %6.2f %6.2f BAD_AZEL", act_az2, act_el);
        }
    }

    // STATUS LINE 4 - lower left
    str4[0] = '\0';

    // STATUS LINE 5 - lower left
    //    ADJ adj_az adj_el
    sprintf(str5, "ADJ %.2f %.2f", MSTEP_TO_AZDEG(adj_az_mstep), MSTEP_TO_ELDEG(adj_el_mstep));

    // STATUS LINE 6 - lower left
    //    CONNECTED mmm:ss
    if (connection_established_time_us != 0) {
        int minutes, secs;
        secs = (microsec_timer() - connection_established_time_us) / 1000000;
        minutes = secs / 60;
        secs -= minutes * 60;
        sprintf(str6, "CONNECTED %d:%2.2d", minutes, secs);
    }

    // unlock mutex
    pthread_mutex_unlock(&tele_ctrl_mutex);
}

// return true if the telescope mechanism can be positioned to az/el
static bool tele_ctrl_is_azel_valid(double az, double el)
{
    if (el < 0 || el > 90) {
        return false;
    }

    if (max_valid_az >= min_valid_az) {
        if (az < min_valid_az || az > max_valid_az) {
            return false;
        }
    } else {
        if (az < min_valid_az && az > max_valid_az) {
            return false;
        }
    }
    
    return true;
}

// -----------------  TELE PANE HNDLR  ------------------------------------

// structure for camera digital pan and zoom
// - image_width, image_height: the size of the texture
// - skip_cols, skip_rows: the number of columns and rows to skip
//   to locate the top left of the image; used in call to 
//   sdl_update_xxx_texture 
// - image_x_ctr, image_y_ctr: these control the pan location, and are
//   updated by SDL_EVENT_TELE_MOUSE_MOTION
// - image_scale: controls the zoom, and is updated by SDL_EVENT_TELE_MOUSE_WHEEL
// - cam_width, cam_height: these are used to check for a change in the 
//   actual camera width/height (found in cam_img), and if change is detected
//   the image_scale, and image_x/y_ctr are reset
static struct {
    int image_width;
    int image_height;
    int skip_cols;
    int skip_rows;
    int image_x_ctr;
    int image_y_ctr;
    double image_scale;
    int cam_width;
    int cam_height;
} pz;

int tele_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event)
{
    struct {
        int none;
    } * vars = pane_cx->vars;
    rect_t * pane = &pane_cx->pane;

    // ----------------------------
    // -------- INITIALIZE --------
    // ----------------------------

    if (request == PANE_HANDLER_REQ_INITIALIZE) {
        vars = pane_cx->vars = calloc(1,sizeof(*vars));
        DEBUG("PANE x,y,w,h  %d %d %d %d\n",
            pane->x, pane->y, pane->w, pane->h);
        return PANE_HANDLER_RET_NO_ACTION;
    }

    // ------------------------
    // -------- RENDER --------
    // ------------------------

    if (request == PANE_HANDLER_REQ_RENDER) {
        int fontsz, sdlpr_row, sdlpr_col, i;
        int required_cam_texture_signature;
        char str1[100], str2[100], str3[100], str4[100], str5[100], str6[100];
        double frames_per_sec;
        uint64_t time_now_us;

        static texture_t cam_texture = NULL;
        static int cam_texture_signature = 0;

        fontsz = 20;  // tele pane
        frames_per_sec = 0;

        // display camera image ...

        // lock cam_img struct
        // if image avail
        //     sanitize the pan/zoom control info
        //     create texture
        //     copy pixels to the texture
        //     unlock cam_img struct
        //     render the texture
        // else
        //     unlock cam_img struct
        //     display 'no image'
        // endif

        // lock cam_img struct
        pthread_mutex_lock(&cam_img_mutex);

        // if image avail
        if (CAM_IMG_VALID) {
            // sanitize the pan/zoom control info
            sanitize_pan_zoom();

            // create texture
            required_cam_texture_signature = (cam_img.pixel_fmt << 28) |
                                             (pz.image_width    << 14) |
                                             (pz.image_height   <<  0);
            if (cam_texture_signature != required_cam_texture_signature) {
                sdl_destroy_texture(cam_texture);

                if (cam_img.pixel_fmt == PIXEL_FMT_IYUV) {
                    cam_texture = sdl_create_iyuv_texture(pz.image_width, pz.image_height);
                } else if (cam_img.pixel_fmt == PIXEL_FMT_YUY2) {
                    cam_texture = sdl_create_yuy2_texture(pz.image_width, pz.image_height);
                } else {
                    FATAL("BUG: cam_img.pixel_fmt = %d\n", cam_img.pixel_fmt);
                }
                if (cam_texture == NULL) {
                    FATAL("failed to create cam_texture\n");
                }

                cam_texture_signature = required_cam_texture_signature;
            }

            // copy pixels to the texture
            if (cam_img.pixel_fmt == PIXEL_FMT_IYUV) {
                unsigned char *y_plane, *u_plane, *v_plane;
                int y_pitch, u_pitch, v_pitch;

                y_plane = cam_img.pixels;
                y_pitch = cam_img.width;
                u_plane = y_plane + (cam_img.width * cam_img.height);
                u_pitch = cam_img.width/2;
                v_plane = u_plane + ((cam_img.width/2) * (cam_img.height/2));
                v_pitch = cam_img.width/2;

                DEBUG("imgx,y,w,h = %d %d %d %d  sr=%d sc=%d\n",
                      pz.image_x_ctr, pz.image_y_ctr, pz.image_width, pz.image_height,
                      pz.skip_rows, pz.skip_cols);

                y_plane += pz.skip_rows * cam_img.width + pz.skip_cols;
                u_plane += (pz.skip_rows / 2) * (cam_img.width / 2) + (pz.skip_cols / 2);
                v_plane += (pz.skip_rows / 2) * (cam_img.width / 2) + (pz.skip_cols / 2);

                sdl_update_iyuv_texture(cam_texture, y_plane, y_pitch, u_plane, u_pitch, v_plane, v_pitch);
            } else if (cam_img.pixel_fmt == PIXEL_FMT_YUY2) {
                sdl_update_yuy2_texture(
                    cam_texture,
                    cam_img.pixels + (pz.skip_rows * cam_img.width * 2) + (pz.skip_cols * 2),
                    cam_img.width * 2);
            } else {
                FATAL("BUG: cam_img.pixel_fmt = %d\n", cam_img.pixel_fmt);
            }

            // determine the frame receive rate
            frames_per_sec = 0;
            time_now_us = microsec_timer();
            for (i = 0; i < MAX_CAM_IMG_RECV_TIME_US; i++) {
                if (time_now_us - cam_img.recv_time_us[i] < 3000000) {
                    frames_per_sec++;
                } else {
                    break;
                }
            }
            frames_per_sec /= 3;

            // unlock cam_img struct
            pthread_mutex_unlock(&cam_img_mutex);

            // render the texture
            rect_t loc = {0, 0, pane->w, pane->h};
            sdl_render_scaled_texture(pane, &loc, cam_texture);
        } else {
            // unlock cam_img struct
            pthread_mutex_unlock(&cam_img_mutex);

            // display 'no image'
            sdlpr_col = (sdl_pane_cols(pane,fontsz) - 15) / 2;
            if (sdlpr_col < 0) sdlpr_col = 0;
            sdlpr_row = sdl_pane_rows(pane,fontsz) / 2;
            SDLPR("NO CAMERA IMAGE");
        }

        // display status lines that are supplied by tele_ctlr_get_status
        // - str1,2,3 at upper left
        // - str4,5,6 at lower left
        tele_ctrl_get_status(str1, str2, str3, str4, str5, str6);
        sdl_render_printf(pane, COL2X(0,fontsz), ROW2Y(0,fontsz), fontsz, WHITE, BLACK, "%s", str1);
        sdl_render_printf(pane, COL2X(0,fontsz), ROW2Y(1,fontsz), fontsz, WHITE, BLACK, "%s", str2);
        sdl_render_printf(pane, COL2X(0,fontsz), ROW2Y(2,fontsz), fontsz, WHITE, BLACK, "%s", str3);
        sdl_render_printf(pane, COL2X(0,fontsz), pane->h-ROW2Y(3,fontsz), fontsz, WHITE, BLACK, "%s", str4);
        sdl_render_printf(pane, COL2X(0,fontsz), pane->h-ROW2Y(2,fontsz), fontsz, WHITE, BLACK, "%s", str5);
        sdl_render_printf(pane, COL2X(0,fontsz), pane->h-ROW2Y(1,fontsz), fontsz, WHITE, BLACK, "%s", str6);

        // display received frames per second rate str, lower right
        sdl_render_printf(pane, pane->w-COL2X(8,fontsz), pane->h-ROW2Y(1,fontsz), fontsz, WHITE, BLACK, 
            "%0.1g FPS %c ", frames_per_sec, "\\|/-"[cam_img.recv_count%4]);

        // register mouse motion and wheel events to support camera digital pan/zoom
        rect_t loc = {0,0,pane->w,pane->h};
        sdl_register_event(pane, &loc, SDL_EVENT_TELE_MOUSE_MOTION, SDL_EVENT_TYPE_MOUSE_MOTION, pane_cx);
        sdl_register_event(pane, &loc, SDL_EVENT_TELE_MOUSE_WHEEL, SDL_EVENT_TYPE_MOUSE_WHEEL, pane_cx);

        // register control events 
        // - row 0
        if (connected) {
            sdl_render_text_and_register_event(
                pane, pane->w-COL2X(9,fontsz), ROW2Y(0,fontsz), fontsz, 
                motors != MOTORS_CLOSED ? "MTR_CLOSE" : "MTR_OPEN",
                LIGHT_BLUE, BLACK, 
                motors != MOTORS_CLOSED ? SDL_EVENT_TELE_MOTORS_CLOSE : SDL_EVENT_TELE_MOTORS_OPEN,
                SDL_EVENT_TYPE_MOUSE_CLICK, pane_cx);
        }
        // - row 1
        if (motors == MOTORS_OPEN) {
            sdl_render_text_and_register_event(
                pane, pane->w-COL2X(9,fontsz), ROW2Y(1,fontsz), fontsz, 
                calibrated ? "UN_CAL" : "CALIBRATE",
                LIGHT_BLUE, BLACK, 
                calibrated ? SDL_EVENT_TELE_UN_CALIBRATE : SDL_EVENT_TELE_CALIBRATE,
                SDL_EVENT_TYPE_MOUSE_CLICK, pane_cx);
        }
        // - row 2
        if (calibrated && tgt_azel_valid) {
            sdl_render_text_and_register_event(
                pane, pane->w-COL2X(9,fontsz), ROW2Y(2,fontsz), fontsz, 
                tracking_enabled ? "TRACK_DIS" : "TRACK_ENA",
                LIGHT_BLUE, BLACK, 
                tracking_enabled ? SDL_EVENT_TELE_TRK_DISABLE : SDL_EVENT_TELE_TRK_ENABLE,
                SDL_EVENT_TYPE_MOUSE_CLICK, pane_cx);
        } else if (motors == MOTORS_CLOSED) {
            sdl_render_text_and_register_event(
                pane, pane->w-COL2X(9,fontsz), ROW2Y(2,fontsz), fontsz, 
                "SHDN_CTLR",
                LIGHT_BLUE, BLACK, 
                SDL_EVENT_TELE_SHUTDN_CTLR,
                SDL_EVENT_TYPE_MOUSE_CLICK, pane_cx);
        }

        return PANE_HANDLER_RET_NO_ACTION;
    }

    // -----------------------
    // -------- EVENT --------
    // -----------------------

    if (request == PANE_HANDLER_REQ_EVENT) {
        int ret;

        // some of the event_ids are handled here, and the majority
        // of the event_ids are handled by call to tele_ctrl_process_cmd
        if (event->event_id == SDL_EVENT_TELE_MOUSE_MOTION) {
            if (CAM_IMG_VALID) {
                pz.image_x_ctr -= event->mouse_motion.delta_x;
                pz.image_y_ctr -= event->mouse_motion.delta_y;
            }
        } else if (event->event_id == SDL_EVENT_TELE_MOUSE_WHEEL) {
            if (CAM_IMG_VALID) {
                int dy = event->mouse_wheel.delta_y;
                if (dy < 0 && pz.image_scale < 1) {
                    pz.image_scale *= 1.1;
                    if (pz.image_scale > 1) pz.image_scale = 1;
                }
                if (dy > 0 && pz.image_scale > .1) {
                    pz.image_scale /= 1.1;
                }
            }
        } else {
            tele_ctrl_process_cmd(event->event_id);
        }

        // for arrow keys do not redraw the displays because they occur rapidly
        // when they key is held, and the arrow key events will accumulate, so that
        // when the key is released the events continue for a period of time; by not
        // redrawing the display the arrow keys can be processed as they are received
        // and do not accumulate
        if (event->event_id == SDL_EVENT_KEY_UP_ARROW ||
            event->event_id == SDL_EVENT_KEY_DOWN_ARROW ||
            event->event_id == SDL_EVENT_KEY_LEFT_ARROW ||
            event->event_id == SDL_EVENT_KEY_RIGHT_ARROW ||
            event->event_id == SDL_EVENT_KEY_SHIFT_UP_ARROW ||
            event->event_id == SDL_EVENT_KEY_SHIFT_DOWN_ARROW ||
            event->event_id == SDL_EVENT_KEY_SHIFT_LEFT_ARROW ||
            event->event_id == SDL_EVENT_KEY_SHIFT_RIGHT_ARROW)
        {
            ret = PANE_HANDLER_RET_NO_ACTION;
        } else {
            ret = PANE_HANDLER_RET_DISPLAY_REDRAW;
        }

        return ret;
    }

    // ---------------------------
    // -------- TERMINATE --------
    // ---------------------------

    if (request == PANE_HANDLER_REQ_TERMINATE) {
        free(vars);
        return PANE_HANDLER_RET_NO_ACTION;
    }

    // not reached
    assert(0);
    return PANE_HANDLER_RET_NO_ACTION;
}

static void sanitize_pan_zoom(void)
{
    // sanity check image_scale, must be <= 1
    if (pz.image_scale > 1) {
        FATAL("BUG: image_scale %f is greater than 1\n", pz.image_scale);
    }

    // if camera width or height has changed then
    // reset image_scale
    if (pz.cam_width != cam_img.width || pz.cam_height != cam_img.height) {
        pz.image_scale = 1;
        pz.cam_width = cam_img.width;
        pz.cam_height = cam_img.height;
    }

    // determine image_width and image_height
    pz.image_width  = pz.cam_width * pz.image_scale;
    pz.image_height = pz.cam_height * pz.image_scale;

    // sanitize x/y center to ensure that the image_width
    // and image_height don't extend off the boundary
    if (pz.image_x_ctr - pz.image_width/2 < 0) {
        pz.image_x_ctr = pz.image_width/2;
    }
    if (pz.image_x_ctr + pz.image_width/2 >= pz.cam_width) {
        pz.image_x_ctr = pz.cam_width - pz.image_width/2;
    }
    if (pz.image_y_ctr - pz.image_height/2 < 0) {
        pz.image_y_ctr = pz.image_height/2;
    }
    if (pz.image_y_ctr + pz.image_height/2 >= pz.cam_height) {
        pz.image_y_ctr = pz.cam_height - pz.image_height/2;
    }

    // determine pixel rows and cols to skip when passing
    // pixels to sdl_update_xxx_texture
    pz.skip_cols = pz.image_x_ctr - pz.image_width/2;
    pz.skip_rows = pz.image_y_ctr - pz.image_height/2;

    // the sdl_update_yuy2_texture does not work well when
    // skip_cols is odd; so if skip_cols is odd decrement 
    // it by 1
    if (pz.skip_cols & 1) pz.skip_cols -= 1;
}

// -----------------  TELE CAM INFO PANE HNDLR  ---------------------------

// XXX scroll arrows or mouse motion for moving the menu if it has too many entries to fit in the pane

int tele_cam_info_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event)
{
    struct {
        int                 selected;
        uint64_t            selected_time_us;
        char                cam_ctrls_buff[10000];
        cam_query_ctrls_t * cam_ctrls;
        int                 cam_ctrls_len;
        uint64_t            cam_ctrls_time_us;
    } * vars = pane_cx->vars;
    rect_t * pane = &pane_cx->pane;

    #define SDL_EVENT_TELE_CAM_INFO_MOUSE_WHEEL          (SDL_EVENT_USER_DEFINED + 0)
    #define SDL_EVENT_TELE_CAM_INFO_CAM_RESET_REQ        (SDL_EVENT_USER_DEFINED + 1)
    #define SDL_EVENT_TELE_CAM_INFO_RESET_DEFAULT_VALUES (SDL_EVENT_USER_DEFINED + 2)
    #define SDL_EVENT_TELE_CAM_INFO_CLEAR_SELECTED       (SDL_EVENT_USER_DEFINED + 3)
    #define SDL_EVENT_TELE_CAM_INFO_CTRLS_BASE           (SDL_EVENT_USER_DEFINED + 10)

    // ----------------------------
    // -------- INITIALIZE --------
    // ----------------------------

    if (request == PANE_HANDLER_REQ_INITIALIZE) {
        vars = pane_cx->vars = calloc(1,sizeof(*vars));
        vars->selected = -1;
        vars->cam_ctrls = (void*)vars->cam_ctrls_buff;
        DEBUG("PANE x,y,w,h  %d %d %d %d\n",
            pane->x, pane->y, pane->w, pane->h);
        return PANE_HANDLER_RET_NO_ACTION;
    }

    // ------------------------
    // -------- RENDER --------
    // ------------------------

    if (request == PANE_HANDLER_REQ_RENDER) {
        int fontsz, sdlpr_row, sdlpr_col;

        fontsz = 20;  // tele cam info pane

        // register CLEAR_SELECTED first, this event is the full pane
        rect_t loc = {0,0,pane->w,pane->h};
        sdl_register_event(pane, &loc, SDL_EVENT_TELE_CAM_INFO_CLEAR_SELECTED, SDL_EVENT_TYPE_MOUSE_CLICK, pane_cx);

        // title
        sdlpr_col = 12; sdlpr_row = 0; SDLPR("CAMERA CTRLS");

        // make a safe copy of cam_ctrls;
        // the remainder of this routine should only refer to this copy
        pthread_mutex_lock(&cam_ctrls_mutex);
        memcpy(vars->cam_ctrls, cam_ctrls, cam_ctrls_len);
        vars->cam_ctrls_len = cam_ctrls_len;
        vars->cam_ctrls_time_us = cam_ctrls_time_us;
        pthread_mutex_unlock(&cam_ctrls_mutex);

        // if camera controls are available then
        //   display the camera controls
        // else
        //   display 'UNAVAILABLE'
        // endif
        if (vars->cam_ctrls->max_cam_ctrl != 0 && 
            vars->cam_ctrls_len != 0 &&
            CAM_CTRLS_VALID)
        {
            int32_t i, j;
            char *strings, *s, *menu_strings[100], current_value_str[100];
            char cam_ctrl_str[100];

            // clear selected if it is for a button event and has been set for 1 second
            if (vars->selected != -1 && 
                vars->cam_ctrls->cam_ctrl[vars->selected].type == CAM_CTRL_TYPE_BUTTON &&
                microsec_timer() - vars->selected_time_us > 1000000)
            {
                vars->selected = -1;
            }

            // loop over the cam ctrls
            strings = (char*)&vars->cam_ctrls->cam_ctrl[vars->cam_ctrls->max_cam_ctrl];
            for (i = 0; i < vars->cam_ctrls->max_cam_ctrl; i++) {
                struct cam_ctrl_s *x = &vars->cam_ctrls->cam_ctrl[i];

                // build table of menu_strings or this cam ctrl
                s = strings + x->menu_strings_offset;
                for (j = 0; j < x->menu_strings_count; j++) {
                    menu_strings[j] = s;
                    s += strlen(s) + 1;
                }

                // construct the current_value_str, if menu_strings are available
                // then use the appropriate menu_string, otherwise convert 
                // current_value to number
                if (x->current_value == NO_CAM_VALUE) {
                    sprintf(current_value_str, "NO_CAM_VALUE");
                } else if (x->menu_strings_count > 0 &&
                           x->current_value >= 0 && x->current_value < x->menu_strings_count)
                {
                    sprintf(current_value_str, "%s", menu_strings[x->current_value]);
                } else {
                    sprintf(current_value_str, "%d", x->current_value);
                }

                // create the string that will be displayed for the cam_ctrl
                // currently being processed
                if (x->type == CAM_CTRL_TYPE_BUTTON) {
                    sprintf(cam_ctrl_str, "BUTTON %s", x->name);
                } else if (x->type == CAM_CTRL_TYPE_READ_WRITE) {
                    sprintf(cam_ctrl_str, "RDWR   %s = %s", x->name, current_value_str);
                } else if (x->type == CAM_CTRL_TYPE_READ_ONLY) {
                    sprintf(cam_ctrl_str, "RDONLY %s = %s", x->name, current_value_str);
                } else {
                    sprintf(cam_ctrl_str, "OTHER  %s", x->name);
                }
    
                // if the cam_ctrl can be written then register an event,
                // otherwise just display
                if (x->type == CAM_CTRL_TYPE_BUTTON || 
                    x->type == CAM_CTRL_TYPE_READ_WRITE)
                {
                    sdl_render_text_and_register_event(
                        pane, COL2X(0,fontsz), ROW2Y(2+i,fontsz), fontsz, 
                        cam_ctrl_str,
                        vars->selected == i ? GREEN : LIGHT_BLUE, 
                        BLACK, 
                        SDL_EVENT_TELE_CAM_INFO_CTRLS_BASE+i,
                        SDL_EVENT_TYPE_MOUSE_CLICK, pane_cx);
                } else {
                    sdl_render_printf(
                        pane, COL2X(0,fontsz), ROW2Y(2+i,fontsz), fontsz, 
                        WHITE, BLACK, "%s", cam_ctrl_str);
                }
            }

            // if a READ_WRITE ctrl is selected then register MOUSE_WHEEL event, which
            // will be used to update the selected cam ctrl value
            if (vars->selected != -1 && 
                vars->cam_ctrls->cam_ctrl[vars->selected].type == CAM_CTRL_TYPE_READ_WRITE)
            {
                rect_t loc = {0,0,pane->w,pane->h};
                sdl_register_event(pane, &loc, SDL_EVENT_TELE_CAM_INFO_MOUSE_WHEEL, SDL_EVENT_TYPE_MOUSE_WHEEL, pane_cx);
            }

            // register RESET_DEFAULT_VALUES event
            sdl_render_text_and_register_event(
                pane, COL2X(0,fontsz), pane->h - ROW2Y(1,fontsz), fontsz, 
                "RESET_DEFAULT_VALUES",
                LIGHT_BLUE, BLACK, 
                SDL_EVENT_TELE_CAM_INFO_RESET_DEFAULT_VALUES, 
                SDL_EVENT_TYPE_MOUSE_CLICK, pane_cx);

            // register CAM_RESET_REQ event
            sdl_render_text_and_register_event(
                pane, pane->w-COL2X(5,fontsz), pane->h - ROW2Y(1,fontsz), fontsz, 
                "RESET",
                LIGHT_BLUE, BLACK, 
                SDL_EVENT_TELE_CAM_INFO_CAM_RESET_REQ, 
                SDL_EVENT_TYPE_MOUSE_CLICK, pane_cx);
        } else {
            sdlpr_col = 12; sdlpr_row = 4; SDLPR("NOT AVAILABLE");
            vars->selected = -1;
        }

        return PANE_HANDLER_RET_NO_ACTION;
    }

    // -----------------------
    // -------- EVENT --------
    // -----------------------

    if (request == PANE_HANDLER_REQ_EVENT) {
        switch (event->event_id) {
        case SDL_EVENT_TELE_CAM_INFO_CTRLS_BASE+0 ... SDL_EVENT_TELE_CAM_INFO_CTRLS_BASE+100:
            // keep track of which cam ctrl is selected
            vars->selected = event->event_id - SDL_EVENT_TELE_CAM_INFO_CTRLS_BASE;
            vars->selected_time_us = microsec_timer();

            // if a BUTTON type control was selected then send msg 
            // to process the button
            struct cam_ctrl_s *x = &vars->cam_ctrls->cam_ctrl[vars->selected];
            if (x->type == CAM_CTRL_TYPE_BUTTON) {
                char msg_buffer[100];
                msg_t * msg = (msg_t*)msg_buffer;
                msg_cam_ctrls_set_t * msg_cam_ctrls_set = (void*)msg->data;

                msg->id = MSGID_CAM_CTRLS_SET;
                msg->data_len = sizeof(msg_cam_ctrls_set_t);
                msg_cam_ctrls_set->cid = vars->cam_ctrls->cam_ctrl[vars->selected].cid;
                msg_cam_ctrls_set->value = 0;
                comm_send_msg(msg);
            }
            break;
        case SDL_EVENT_TELE_CAM_INFO_CLEAR_SELECTED:
            // clear selected
            vars->selected = -1;
            vars->selected_time_us = 0;
            break;
        case SDL_EVENT_TELE_CAM_INFO_MOUSE_WHEEL: {
            int dy, cid;
            char msg_buffer[100];
            msg_t * msg = (msg_t*)msg_buffer;
            msg_cam_ctrls_incr_decr_t * msg_cam_ctrls_incr_decr = (void*)msg->data;

            // this event should only occur when selected is set to a READ_WRITE value;
            // start by making some sanity checks
            dy = event->mouse_wheel.delta_y;
            if (dy == 0) {
                ERROR("dy shouldn't be 0\n");
                break;
            }
            if (vars->selected == -1 || 
                vars->selected >= vars->cam_ctrls->max_cam_ctrl ||
                vars->cam_ctrls->cam_ctrl[vars->selected].type != CAM_CTRL_TYPE_READ_WRITE) 
            {
                ERROR("selected %d is invalid\n", vars->selected);
                break;
            }

            // get the selected control id (cid)
            cid = vars->cam_ctrls->cam_ctrl[vars->selected].cid;
            DEBUG("selected=%d  cid=0x%x  dy=%d\n",  vars->selected, cid, dy);

            // send message to increment or decrement the cid
            msg->id = MSGID_CAM_CTRLS_INCR_DECR;
            msg->data_len = sizeof(msg_cam_ctrls_incr_decr_t);
            msg_cam_ctrls_incr_decr->cid = cid;
            msg_cam_ctrls_incr_decr->incr_flag = (dy > 0);
            comm_send_msg(msg);
            break; }
        case SDL_EVENT_TELE_CAM_INFO_RESET_DEFAULT_VALUES: {
            // send message to have the webcam reset to default values
            msg_t msg = { MSGID_CAM_CTRLS_RESET, 0 };
            comm_send_msg(&msg);
            break; }
        case SDL_EVENT_TELE_CAM_INFO_CAM_RESET_REQ: {
            // send message to have the webcam reset
            msg_t msg = { MSGID_CAM_RESET_REQ, 0 };
            comm_send_msg(&msg);
            break; }
        }
        return PANE_HANDLER_RET_NO_ACTION;
    }

    // ---------------------------
    // -------- TERMINATE --------
    // ---------------------------

    if (request == PANE_HANDLER_REQ_TERMINATE) {
        free(vars);
        return PANE_HANDLER_RET_NO_ACTION;
    }

    // not reached
    assert(0);
    return PANE_HANDLER_RET_NO_ACTION;
}

// -----------------  TELE MOTOR INFO PANE HNDLR  -------------------------

int tele_motor_info_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event)
{
    struct {
        int none;
    } * vars = pane_cx->vars;
    rect_t * pane = &pane_cx->pane;

    // ----------------------------
    // -------- INITIALIZE --------
    // ----------------------------

    if (request == PANE_HANDLER_REQ_INITIALIZE) {
        vars = pane_cx->vars = calloc(1,sizeof(*vars));
        DEBUG("PANE x,y,w,h  %d %d %d %d\n",
            pane->x, pane->y, pane->w, pane->h);
        return PANE_HANDLER_RET_NO_ACTION;
    }

    // ------------------------
    // -------- RENDER --------
    // ------------------------

    if (request == PANE_HANDLER_REQ_RENDER) {
        int fontsz, sdlpr_row, sdlpr_col;
        struct motor_status_s * m0 = &ctlr_motor_status.motor[0];
        struct motor_status_s * m1 = &ctlr_motor_status.motor[1];
        char   error_status_str0[100], error_status_str1[100];
        char  *saveptr0, *saveptr1, *errsts0, *errsts1;
        int    errsts_cnt=0;

        fontsz = 20;  // tele motor info pane

        // display motor variables...

        // title
        sdlpr_col = 12; sdlpr_row = 0; SDLPR("MOTOR INFO");

        // if motor vars available then
        //   display the motor variables
        // else
        //   display 'UNAVAILABLE'
        // endif
        if (CTLR_MOTOR_STATUS_VALID) {
            sdlpr_col = 0; sdlpr_row = 2;
            SDLPR("OPENED   %9d %9d", m0->opened, m1->opened);
            SDLPR("ENERG    %9d %9d", m0->energized, m1->energized);
            SDLPR("VOLTAGE  %9d %9d", m0->vin_voltage_mv, m1->vin_voltage_mv);
            SDLPR("CURR_POS %9d %9d", m0->curr_pos_mstep, m1->curr_pos_mstep);
            SDLPR("TGT_POS  %9d %9d", m0->tgt_pos_mstep, m1->tgt_pos_mstep);
            SDLPR("CURR_VEL %9.1f %9.1f", m0->curr_vel_mstep_per_sec, m1->curr_vel_mstep_per_sec);
            SDLPR("OP_STATE %9s %9s",     m0->operation_state_str, m1->operation_state_str);
            while (true) {
                if (errsts_cnt == 0) {
                    strcpy(error_status_str0, m0->error_status_str);
                    strcpy(error_status_str1, m1->error_status_str);
                    errsts0 = strtok_r(error_status_str0, " ", &saveptr0);
                    errsts1 = strtok_r(error_status_str1, " ", &saveptr1);
                }

                if (errsts0 == NULL) errsts0 = " ";
                if (errsts1 == NULL) errsts1 = " ";
                SDLPR("%8s %9s %9s", errsts_cnt == 0 ? "ERR_STAT" : "", errsts0, errsts1);

                errsts0 = strtok_r(NULL, " ", &saveptr0);
                errsts1 = strtok_r(NULL, " ", &saveptr1);
                if ((errsts0 == NULL && errsts1 == NULL) || ++errsts_cnt == 5) {
                    break;
                }
            }
        } else {
            sdlpr_col = 12; sdlpr_row = 4; SDLPR("NOT AVAILABLE");
        }

        return PANE_HANDLER_RET_NO_ACTION;
    }

    // -----------------------
    // -------- EVENT --------
    // -----------------------

    if (request == PANE_HANDLER_REQ_EVENT) {
        return PANE_HANDLER_RET_NO_ACTION;
    }

    // ---------------------------
    // -------- TERMINATE --------
    // ---------------------------

    if (request == PANE_HANDLER_REQ_TERMINATE) {
        free(vars);
        return PANE_HANDLER_RET_NO_ACTION;
    }

    // not reached
    assert(0);
    return PANE_HANDLER_RET_NO_ACTION;
}
