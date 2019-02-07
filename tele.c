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

//#define TEST_WITH_ONLY_AZ_MOTOR

//
// typedefs
//

//
// variables
//

// comm to tele ctlr vars
int   sfd = -1;
bool  connected;

// telescope motor status vars
msg_status_data_t ctlr_motor_status;
long              ctlr_motor_status_us;

// telsecope control vars
int    motors;
bool   calibrated;
bool   tracking_enabled;

int    cal_az0_mstep;
int    cal_el0_mstep;

double tgt_az, tgt_el;
bool   tgt_azel_valid;

double act_az, act_el;
bool   act_azel_available;
bool   act_azel_valid;

int    adj_az_mstep, adj_el_mstep;

#ifdef TEST_WITH_ONLY_AZ_MOTOR
int simulate_curr_el_mstep;
#endif

// general vars
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

//
// prototypes
//

void * comm_thread(void * cx);
void * comm_heartbeat_thread(void * cx);
int comm_process_recvd_msg(msg_t * msg);
void comm_send_msg(msg_t * msg);

void * tele_ctrl_thread(void * cx);
void tele_ctrl_process_cmd(int event_id);
void tele_ctrl_get_status(char *str1, char *str2, char *str3);
bool tele_ctrl_is_azel_valid(double az, double el);

// -----------------  TELE INIT  ------------------------------------------

int tele_init(char *incl_ss_obj_str)
{
    pthread_t thread_id;

    // create threads
    pthread_create(&thread_id, NULL, comm_thread, NULL);
    pthread_create(&thread_id, NULL, comm_heartbeat_thread, NULL);
    pthread_create(&thread_id, NULL, tele_ctrl_thread, NULL);

    // return success
    return 0;
}

// -----------------  COMM TO TELE CTLR  ----------------------------------

void * comm_thread(void * cx)
{
    int rc, sfd_temp, len;
    struct sockaddr_in addr;
    struct timeval rcvto = {1, 0};  // sec, usec
    char msg_buffer[1000];
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

int comm_process_recvd_msg(msg_t * msg)
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
#ifdef TEST_WITH_ONLY_AZ_MOTOR
        // when simulating the el motor override the received el motor 
        // position with the simulated value
        ctlr_motor_status.motor[1].curr_pos_mstep = simulate_curr_el_mstep;
#endif
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

void comm_send_msg(msg_t * msg)
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

void * comm_heartbeat_thread(void * cx)
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
}

// -----------------  TELE CONTROL  ---------------------------------------

void * tele_ctrl_thread(void * cx)
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
#ifdef TEST_WITH_ONLY_AZ_MOTOR
        } else if (ctlr_motor_status.motor[0].opened == 1 &&
                   strcmp(ctlr_motor_status.motor[0].operation_state_str, "NORMAL") == 0 &&
                   strcmp(ctlr_motor_status.motor[0].error_status_str, "NO_ERR") == 0)
        {
            motors = MOTORS_OPEN;
#else
        } else if (ctlr_motor_status.motor[0].opened == 1 &&
                   strcmp(ctlr_motor_status.motor[0].operation_state_str, "NORMAL") == 0 &&
                   strcmp(ctlr_motor_status.motor[0].error_status_str, "NO_ERR") == 0 &&
                   ctlr_motor_status.motor[1].opened == 1 &&
                   strcmp(ctlr_motor_status.motor[1].operation_state_str, "NORMAL") == 0 &&
                   strcmp(ctlr_motor_status.motor[1].error_status_str, "NO_ERR") == 0)
        {
            motors = MOTORS_OPEN;
#endif
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
        tgt_azel_valid = tele_ctrl_is_azel_valid(tgt_az, tgt_el);

        // determine act_az/el from ctrl_motor_status shaft position
        if (calibrated) {
            int curr_az_mstep = ctlr_motor_status.motor[0].curr_pos_mstep;
            int curr_el_mstep = ctlr_motor_status.motor[1].curr_pos_mstep;
            act_az = MSTEP_TO_AZDEG(curr_az_mstep - cal_az0_mstep - adj_az_mstep);
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
            int curr_az_mstep = ctlr_motor_status.motor[0].curr_pos_mstep;

            // determine az/el microstep motor positions based on caliabration
            az_mstep = cal_az0_mstep + AZDEG_TO_MSTEP(tgt_az) + adj_az_mstep;
            el_mstep = cal_el0_mstep + ELDEG_TO_MSTEP(tgt_el) + adj_el_mstep;

#ifdef TEST_WITH_ONLY_AZ_MOTOR
            // when simulating the el motor, set simulated_curr_el_mstep to
            // the target el_mstep
            simulate_curr_el_mstep = el_mstep;
#endif

            // update az_mstep, if needed, to be shortest movement
            while (az_mstep - curr_az_mstep > AZMSTEP_180_DEG) az_mstep -= AZMSTEP_360_DEG;
            while (az_mstep - curr_az_mstep < -AZMSTEP_180_DEG) az_mstep += AZMSTEP_360_DEG;

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
}

void tele_ctrl_process_cmd(int event_id)
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

void tele_ctrl_get_status(char *str1, char *str2, char *str3)
{
    double tgt_az2, act_az2;

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
    //   TGT az el DISABLED|INVALID|ACQUIRING|ACHIEVED
    if (!connected) {
        strcpy(str1, "DISCONNECTED");
    } else if (motors == MOTORS_CLOSED) {
        strcpy(str1, "MOTORS_CLOSED");
    } else if (motors == MOTORS_ERROR) {
        strcpy(str1, "MOTORS_ERROR");
    } else if (!calibrated) {
        strcpy(str1, "UNCALIBRATED");
    } else if (!tgt_azel_valid) {
        sprintf(str1, "TGT %6.2f %6.2f BAD_AZEL", tgt_az2, tgt_el);
    } else if (!tracking_enabled) {
        sprintf(str1, "TGT %6.2f %6.2f DISABLED", tgt_az2, tgt_el);
    } else if (fabs(act_az2-tgt_az2) > .015 || fabs(act_el-tgt_el) > .015) {
        sprintf(str1, "TGT %6.2f %6.2f ACQUIRING", tgt_az2, tgt_el);
    } else {
        sprintf(str1, "TGT %6.2f %6.2f ACQUIRED", tgt_az2, tgt_el);
    }

    // STATUS LINE 2 - upper left
    //    ip_address      (when not connected)
    //    ACT az el       (when calibrated)
    //    MOTOR xxx xxx   (when not calibrated)
    if (!connected) {
        sprintf(str2, "IPADDR %s", ctlr_ip);
    } else if (act_azel_available) {
        if (act_azel_valid) {
            sprintf(str2, "ACT %6.2f %6.2f", act_az2, act_el);
        } else {
            sprintf(str2, "ACT %6.2f %6.2f BAD_AZEL", act_az2, act_el);
        }
    } else {
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
    }

    // STATUS LINE 3 - lower left
    sprintf(str3, "ADJ %.2f %.2f", MSTEP_TO_AZDEG(adj_az_mstep), MSTEP_TO_ELDEG(adj_el_mstep));

    // unlock mutex
    pthread_mutex_unlock(&mutex);
}

// return true if the telescope mechanism can be positioned to az/el
bool tele_ctrl_is_azel_valid(double az, double el)
{
    bool valid = true;

    // XXX additional azel valid checks will be needed in tele_ctrl_is_azel_valid
    if (el < 0) {
        valid = false;
    }

    return valid;
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
        char str1[100], str2[100], str3[100];

        fontsz = 20;  // tele pane

        // display status lines
        tele_ctrl_get_status(str1, str2, str3);
        sdl_render_printf(pane, COL2X(0,fontsz), ROW2Y(0,fontsz), fontsz, WHITE, BLACK, "%s", str1);
        sdl_render_printf(pane, COL2X(0,fontsz), ROW2Y(1,fontsz), fontsz, WHITE, BLACK, "%s", str2);
        sdl_render_printf(pane, COL2X(0,fontsz), pane->h-ROW2Y(1,fontsz), fontsz, WHITE, BLACK, "%s", str3);

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

            static char error_status_str[2][15];
            static char operation_state_str[2][15];

            strncpy(error_status_str[0], m0->error_status_str, 14);
            strncpy(error_status_str[1], m1->error_status_str, 14);
            strncpy(operation_state_str[0], m0->operation_state_str, 14);
            strncpy(operation_state_str[1], m1->operation_state_str, 14);

            sdlpr_col = 0;
            sdlpr_row = 5;
            SDLPR("OPENED     %14lld %14lld", m0->opened, m1->opened);
            SDLPR("ENERG      %14lld %14lld", m0->energized, m1->energized);
            SDLPR("VOLTAGE    %14lld %14lld", m0->vin_voltage_mv, m1->vin_voltage_mv);
            SDLPR("CURR_POS   %14lld %14lld", m0->curr_pos_mstep, m1->curr_pos_mstep);
            SDLPR("TGT_POS    %14lld %14lld", m0->tgt_pos_mstep, m1->tgt_pos_mstep);
            SDLPR("CURR_VEL   %14.1f %14.1f", m0->curr_vel_mstep_per_sec, m1->curr_vel_mstep_per_sec);
            SDLPR("OP_STATE   %14s %14s",     operation_state_str[0], operation_state_str[1]);
            SDLPR("ERR_STAT   %14s %14s",     error_status_str[0], error_status_str[1]);
        }

        // register control events 
        // - row 0
        if (connected) {
            sdl_render_text_and_register_event(
                pane, pane->w-COL2X(12,fontsz), ROW2Y(0,fontsz), fontsz, 
                motors != MOTORS_CLOSED ? "MOTORS_CLOSE" : "MOTORS_OPEN",
                LIGHT_BLUE, BLACK, 
                motors != MOTORS_CLOSED ? SDL_EVENT_MOTORS_CLOSE : SDL_EVENT_MOTORS_OPEN,
                SDL_EVENT_TYPE_MOUSE_CLICK, pane_cx);
        }
        // - row 1
        if (motors == MOTORS_OPEN) {
            sdl_render_text_and_register_event(
                pane, pane->w-COL2X(12,fontsz), ROW2Y(1,fontsz), fontsz, 
                calibrated ? "UN_CALIBRATE" : "CALIBRATE",
                LIGHT_BLUE, BLACK, 
                calibrated ? SDL_EVENT_UN_CALIBRATE : SDL_EVENT_CALIBRATE,
                SDL_EVENT_TYPE_MOUSE_CLICK, pane_cx);
        }
        // - row 2
        if (calibrated && tgt_azel_valid) {
            sdl_render_text_and_register_event(
                pane, pane->w-COL2X(12,fontsz), ROW2Y(2,fontsz), fontsz, 
                tracking_enabled ? "TRK_DISABLE" : "TRK_ENABLE",
                LIGHT_BLUE, BLACK, 
                tracking_enabled ? SDL_EVENT_TRK_DISABLE : SDL_EVENT_TRK_ENABLE,
                SDL_EVENT_TYPE_MOUSE_CLICK, pane_cx);
        } else if (motors == MOTORS_CLOSED) {
            sdl_render_text_and_register_event(
                pane, pane->w-COL2X(12,fontsz), ROW2Y(2,fontsz), fontsz, 
                "SHUTDN_CTLR",
                LIGHT_BLUE, BLACK, 
                SDL_EVENT_SHUTDN_CTLR,
                SDL_EVENT_TYPE_MOUSE_CLICK, pane_cx);
        }
        // - row last
        sdl_render_text_and_register_event(
            pane, pane->w-COL2X(11,fontsz), pane->h-ROW2Y(1,fontsz), fontsz, 
            "DISP_SELECT",
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

