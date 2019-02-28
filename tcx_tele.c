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

#define SDL_EVENT_MOTORS_CLOSE (SDL_EVENT_USER_DEFINED + 0)
#define SDL_EVENT_MOTORS_OPEN  (SDL_EVENT_USER_DEFINED + 1)
#define SDL_EVENT_UN_CALIBRATE (SDL_EVENT_USER_DEFINED + 2)
#define SDL_EVENT_CALIBRATE    (SDL_EVENT_USER_DEFINED + 3)
#define SDL_EVENT_TRK_DISABLE  (SDL_EVENT_USER_DEFINED + 4)
#define SDL_EVENT_TRK_ENABLE   (SDL_EVENT_USER_DEFINED + 5)
#define SDL_EVENT_SHUTDN_CTLR  (SDL_EVENT_USER_DEFINED + 6)

#define EVENT_ID_STR(x) \
   ((x) == SDL_EVENT_MOTORS_CLOSE          ? "MOTORS_CLOSE"    : \
    (x) == SDL_EVENT_MOTORS_OPEN           ? "MOTORS_OPEN"     : \
    (x) == SDL_EVENT_UN_CALIBRATE          ? "UN_CALIBRATE"    : \
    (x) == SDL_EVENT_CALIBRATE             ? "CALIBRATE"       : \
    (x) == SDL_EVENT_TRK_DISABLE           ? "TRK_DISABLE"     : \
    (x) == SDL_EVENT_TRK_ENABLE            ? "TRK_ENABLE"      : \
    (x) == SDL_EVENT_SHUTDN_CTLR           ? "SHUTDN_CTLR"     : \
    (x) == SDL_EVENT_KEY_LEFT_ARROW        ? "KEY_LEFT_ARROW"  : \
    (x) == SDL_EVENT_KEY_RIGHT_ARROW       ? "KEY_RIGHT_ARROW" : \
    (x) == SDL_EVENT_KEY_UP_ARROW          ? "KEY_UP_ARROW"    : \
    (x) == SDL_EVENT_KEY_DOWN_ARROW        ? "KEY_DOWN_ARROW"  : \
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

#define CTLR_MOTOR_STATUS_VALID (microsec_timer() - ctlr_motor_status_us <= 1000000)

//
// typedefs
//

//
// variables
//

// comm to tele ctlr vars
static int   sfd = -1;
static bool  connected;

// telescope motor status vars
static msg_status_data_t ctlr_motor_status;
static long              ctlr_motor_status_us;

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

// general vars
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

//
// prototypes
//

static void * comm_thread(void * cx);
static void * comm_heartbeat_thread(void * cx);
static int comm_process_recvd_msg(msg_t * msg);
static void comm_send_msg(msg_t * msg);

static void * tele_ctrl_thread(void * cx);
static void tele_ctrl_process_cmd(int event_id);
static void tele_ctrl_get_status(char *str1, char *str2, char *str3, char *str4);
static bool tele_ctrl_is_azel_valid(double az, double el);

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
    struct timeval rcvto = {1, 0};  // sec, usec
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

    // set 1 second timeout for recv
    if (setsockopt(sfd, SOL_SOCKET, SO_RCVTIMEO, &rcvto, sizeof(rcvto)) == -1) {
        FATAL("setsockopt SO_RCVTIMEO, %s", strerror(errno));
    }

    // send connected msg
    memset(msg,0,sizeof(msg_t));
    msg->id = MSGID_CONNECTED;
    comm_send_msg(msg);

    // on new connection should first recv MSGID_CONNECTED
    len = recv(sfd, msg, sizeof(msg_t), MSG_WAITALL);
    if (len != sizeof(msg_t)) {
        ERROR("recvd initial msg with invalid len %d, %s\n", len, strerror(errno));
        goto lost_connection;
    }
    if (msg->id != MSGID_CONNECTED) {
        ERROR("recvd invalid initial msg, id=%lld\n", msg->id);
        goto lost_connection;
    }
    INFO("established connection to telescope\n");
    connected = true;

    // receive msgs from ctlr_ip, and
    // process them
    while (true) {
        // recv msg
        len = recv(sfd, msg, sizeof(msg_t), MSG_WAITALL);
        if (len != sizeof(msg_t)) {
            ERROR("recvd msg with invalid len %d, %s\n", len, strerror(errno));
            break;
        }
        len = recv(sfd, msg->data, msg->datalen, MSG_WAITALL);
        if (len != msg->datalen) {
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
            if (msg->datalen != explen) { \
                ERROR("incorrect datalen %lld in %s\n", msg->datalen, MSGID_STR(msg->id)); \
                break; \
            } \
        } while (0)

    DEBUG("received %s datalen %lld\n", MSGID_STR(msg->id), msg->datalen);

    switch (msg->id) {
    case MSGID_STATUS: {
        CHECK_DATALEN(sizeof(msg_status_data_t));
        ctlr_motor_status = *(msg_status_data_t *)msg->data;
        ctlr_motor_status_us = microsec_timer();
        break; }
    case MSGID_CAM_IMG: {
        // XXX tbd
        INFO("GOT CAM IMG len %lld\n", msg->datalen);
        break; }
    case MSGID_HEARTBEAT:
        CHECK_DATALEN(0);
        break;
    default:
        ERROR("invalid msgid %lld\n", msg->id);
        return -1;
    }

    return 0;
}

static void comm_send_msg(msg_t * msg)
{
    int len;

    if (!connected && msg->id != MSGID_CONNECTED) {
        return;
    }

    DEBUG("sending %s\n", MSGID_STR(msg->id));

    len = send(sfd, msg, sizeof(msg_t)+msg->datalen, MSG_NOSIGNAL);
    if (len != sizeof(msg_t)+msg->datalen) {
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

// -----------------  TELE CONTROL  ---------------------------------------

static void * tele_ctrl_thread(void * cx)
{
    long time_us, time_last_set_pos_us=0;

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
        pthread_mutex_lock(&mutex);

        // determine motor status from ctlr_motor_status
        if (!connected) {
            motors = MOTORS_ERROR;
        } else if (!CTLR_MOTOR_STATUS_VALID) {
            motors = MOTORS_ERROR;
        } else if (ctlr_motor_status.motor[0].opened == 0 && ctlr_motor_status.motor[1].opened == 0) {
            motors = MOTORS_CLOSED;
        } else if (ctlr_motor_status.motor[0].opened == 1 &&
                   strcmp(ctlr_motor_status.motor[0].operation_state_str, "NORMAL") == 0 &&
                   strcmp(ctlr_motor_status.motor[0].error_status_str, "NO_ERR") == 0 &&
                   ctlr_motor_status.motor[1].opened == 1 &&
                   strcmp(ctlr_motor_status.motor[1].operation_state_str, "NORMAL") == 0 &&
                   strcmp(ctlr_motor_status.motor[1].error_status_str, "NO_ERR") == 0)
        {
            motors = MOTORS_OPEN;
        } else {
            motors = MOTORS_ERROR;
        }

        // if telescope is not fully ready then ensure the 
        // calibrated and tracking_enabled flags are clear
        // and zero the az/el adjustment values
        if (!connected || motors != MOTORS_OPEN || !calibrated) {
            calibrated = false;
            tracking_enabled = false;
            adj_az_mstep = adj_el_mstep = 0;
        }

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
            msg->datalen = sizeof(msg_set_pos_all_data_t);
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
        pthread_mutex_unlock(&mutex);
    }

    return NULL;
}

static void tele_ctrl_process_cmd(int event_id)
{
    // lock mutex
    pthread_mutex_lock(&mutex);

    DEBUG("processing event_id %s (0x%x)\n", EVENT_ID_STR(event_id), event_id);

    switch (event_id) {
    case SDL_EVENT_MOTORS_CLOSE:
        if (connected && motors != MOTORS_CLOSED) {
            msg_t msg = { MSGID_CLOSE_ALL, 0 };
            comm_send_msg(&msg);
            calibrated = false;
            tracking_enabled = false;
        }
        break;
    case SDL_EVENT_MOTORS_OPEN:
        if (connected && motors == MOTORS_CLOSED) {
            msg_t msg = { MSGID_OPEN_ALL, 0 };
            comm_send_msg(&msg);
            calibrated = false;
            tracking_enabled = false;
        }
        break;
    case SDL_EVENT_UN_CALIBRATE:
        if (calibrated) {
            calibrated = false;
            tracking_enabled = false;
        }
        break;
    case SDL_EVENT_CALIBRATE:
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
    case SDL_EVENT_TRK_DISABLE:
        if (calibrated) {
            tracking_enabled = false;
        }
        break;
    case SDL_EVENT_TRK_ENABLE:
        if (calibrated) {
            tracking_enabled = true;
        }
        break;
    case SDL_EVENT_SHUTDN_CTLR:
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
            long mstep, max_mstep;

            if (h == 0 ) {
                max_mstep = AZDEG_TO_MSTEP(1);
                mstep = (fine ? sign : sign * AZDEG_TO_MSTEP(0.1));
            } else {
                max_mstep = ELDEG_TO_MSTEP(1);
                mstep = (fine ? sign : sign * ELDEG_TO_MSTEP(0.1));
            }

            msg->id = MSGID_ADV_POS_SINGLE;
            msg->datalen = sizeof(msg_adv_pos_single_data_t);
            msg_adv_pos_single_data->h = h;
            msg_adv_pos_single_data->mstep = mstep;
            msg_adv_pos_single_data->max_mstep = max_mstep;

            comm_send_msg(msg);
        }
        break; }
    }

    // unlock mutex
    pthread_mutex_unlock(&mutex);
}

static void tele_ctrl_get_status(char *str1, char *str2, char *str3, char *str4)
{
    double tgt_az2, act_az2;
    bool acquired = false;

    // preset returns to empty strings
    str1[0] = '\0';
    str2[0] = '\0';
    str3[0] = '\0';
    str4[0] = '\0';

    // lock mutex
    pthread_mutex_lock(&mutex);

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
            sprintf(motor0_pos_str, "%lld", ctlr_motor_status.motor[0].curr_pos_mstep);
        } else {
            strcpy(motor0_pos_str, "-");
        }
        if (CTLR_MOTOR_STATUS_VALID && ctlr_motor_status.motor[1].opened) {
            sprintf(motor1_pos_str, "%lld", ctlr_motor_status.motor[1].curr_pos_mstep);
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
    sprintf(str4, "ADJ %.2f %.2f", MSTEP_TO_AZDEG(adj_az_mstep), MSTEP_TO_ELDEG(adj_el_mstep));

    // unlock mutex
    pthread_mutex_unlock(&mutex);
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

int tele_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event)
{
    struct {
        int display_choice;
    } * vars = pane_cx->vars;
    rect_t * pane = &pane_cx->pane;

    #define DISPLAY_CHOICE_CAMERA 0
    #define DISPLAY_CHOICE_MOTOR_VARIABLES  1
    #define MAX_DISPLAY_CHOICE 2

    #define SDLPR(fmt, args...) \
        do { \
            sdl_render_printf(pane, COL2X(sdlpr_col,fontsz), ROW2Y(sdlpr_row,fontsz), fontsz, WHITE, BLACK, fmt, ## args); \
            sdlpr_row++; \
        } while (0)

    #define SDL_EVENT_DISPLAY_CHOICE   (SDL_EVENT_USER_DEFINED + 100)

    // ----------------------------
    // -------- INITIALIZE --------
    // ----------------------------

    if (request == PANE_HANDLER_REQ_INITIALIZE) {
        vars = pane_cx->vars = calloc(1,sizeof(*vars));
        vars->display_choice = DISPLAY_CHOICE_MOTOR_VARIABLES;
        DEBUG("PANE x,y,w,h  %d %d %d %d\n",
            pane->x, pane->y, pane->w, pane->h);
        return PANE_HANDLER_RET_NO_ACTION;
    }

    // ------------------------
    // -------- RENDER --------
    // ------------------------

    if (request == PANE_HANDLER_REQ_RENDER) {
        int fontsz, sdlpr_row, sdlpr_col;
        char str1[100], str2[100], str3[100], str4[100];

        fontsz = 20;  // tele pane

        // display status lines
        tele_ctrl_get_status(str1, str2, str3, str4);
        sdl_render_printf(pane, COL2X(0,fontsz), ROW2Y(0,fontsz), fontsz, WHITE, BLACK, "%s", str1);
        sdl_render_printf(pane, COL2X(0,fontsz), ROW2Y(1,fontsz), fontsz, WHITE, BLACK, "%s", str2);
        sdl_render_printf(pane, COL2X(0,fontsz), ROW2Y(2,fontsz), fontsz, WHITE, BLACK, "%s", str3);

        sdl_render_printf(pane, COL2X(0,fontsz), pane->h-ROW2Y(1,fontsz), fontsz, WHITE, BLACK, "%s", str4);

        // display either:
        // - camera image
        // - motor variables
        if (vars->display_choice == DISPLAY_CHOICE_CAMERA) {
            sdlpr_col = 10;
            sdlpr_row = 10;
            SDLPR("CAMERA IMAGE HERE"); // XXX tbd
        } else if (vars->display_choice == DISPLAY_CHOICE_MOTOR_VARIABLES) {
            struct motor_status_s * m0 = &ctlr_motor_status.motor[0];
            struct motor_status_s * m1 = &ctlr_motor_status.motor[1];
            char   error_status_str0[100], error_status_str1[100];
            char  *saveptr0, *saveptr1, *errsts0, *errsts1;
            int    errsts_cnt=0;

            sdlpr_col = 0;
            sdlpr_row = 3;
            SDLPR("OPENED   %9lld %9lld", m0->opened, m1->opened);
            SDLPR("ENERG    %9lld %9lld", m0->energized, m1->energized);
            SDLPR("VOLTAGE  %9lld %9lld", m0->vin_voltage_mv, m1->vin_voltage_mv);
            SDLPR("CURR_POS %9lld %9lld", m0->curr_pos_mstep, m1->curr_pos_mstep);
            SDLPR("TGT_POS  %9lld %9lld", m0->tgt_pos_mstep, m1->tgt_pos_mstep);
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
        }

        // register control events 
        // - row 0
        if (connected) {
            sdl_render_text_and_register_event(
                pane, pane->w-COL2X(7,fontsz), ROW2Y(0,fontsz), fontsz, 
                motors != MOTORS_CLOSED ? "MTR_CLS" : "MTR_OPN",
                LIGHT_BLUE, BLACK, 
                motors != MOTORS_CLOSED ? SDL_EVENT_MOTORS_CLOSE : SDL_EVENT_MOTORS_OPEN,
                SDL_EVENT_TYPE_MOUSE_CLICK, pane_cx);
        }
        // - row 1
        if (motors == MOTORS_OPEN) {
            sdl_render_text_and_register_event(
                pane, pane->w-COL2X(7,fontsz), ROW2Y(1,fontsz), fontsz, 
                calibrated ? "UN_CAL" : "CAL",
                LIGHT_BLUE, BLACK, 
                calibrated ? SDL_EVENT_UN_CALIBRATE : SDL_EVENT_CALIBRATE,
                SDL_EVENT_TYPE_MOUSE_CLICK, pane_cx);
        }
        // - row 2
        if (calibrated && tgt_azel_valid) {
            sdl_render_text_and_register_event(
                pane, pane->w-COL2X(7,fontsz), ROW2Y(2,fontsz), fontsz, 
                tracking_enabled ? "TRK_DIS" : "TRK_EN",
                LIGHT_BLUE, BLACK, 
                tracking_enabled ? SDL_EVENT_TRK_DISABLE : SDL_EVENT_TRK_ENABLE,
                SDL_EVENT_TYPE_MOUSE_CLICK, pane_cx);
        } else if (motors == MOTORS_CLOSED) {
            sdl_render_text_and_register_event(
                pane, pane->w-COL2X(7,fontsz), ROW2Y(2,fontsz), fontsz, 
                "SH_CTLR",
                LIGHT_BLUE, BLACK, 
                SDL_EVENT_SHUTDN_CTLR,
                SDL_EVENT_TYPE_MOUSE_CLICK, pane_cx);
        }
        // - row last
        sdl_render_text_and_register_event(
            pane, pane->w-COL2X(7,fontsz), pane->h-ROW2Y(1,fontsz), fontsz, 
            "DSP_SEL",
            LIGHT_BLUE, BLACK, 
            SDL_EVENT_DISPLAY_CHOICE,
            SDL_EVENT_TYPE_MOUSE_CLICK, pane_cx);

        return PANE_HANDLER_RET_NO_ACTION;
    }

    // -----------------------
    // -------- EVENT --------
    // -----------------------

    if (request == PANE_HANDLER_REQ_EVENT) {
        int ret;

        // some of the event_ids are handled here
        if (event->event_id == SDL_EVENT_DISPLAY_CHOICE) {
            vars->display_choice = (vars->display_choice + 1) % MAX_DISPLAY_CHOICE;
        }

        // the remainng event_ids are processed by the tele_ctrl_process_cmd routine
        tele_ctrl_process_cmd(event->event_id);

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

