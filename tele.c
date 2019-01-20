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

#define SDL_EVENT_MOTORS_CLOSE (SDL_EVENT_USER_DEFINED + 0)
#define SDL_EVENT_MOTORS_OPEN  (SDL_EVENT_USER_DEFINED + 1)
#define SDL_EVENT_UN_CALIBRATE (SDL_EVENT_USER_DEFINED + 2)
#define SDL_EVENT_CALIBRATE    (SDL_EVENT_USER_DEFINED + 3)
#define SDL_EVENT_TRK_DISABLE  (SDL_EVENT_USER_DEFINED + 4)
#define SDL_EVENT_TRK_ENABLE   (SDL_EVENT_USER_DEFINED + 5)

#define EVENT_ID_STR(x) \
   ((x) == SDL_EVENT_MOTORS_CLOSE          ? "MOTORS_CLOSE"    : \
    (x) == SDL_EVENT_MOTORS_OPEN           ? "MOTORS_OPEN"     : \
    (x) == SDL_EVENT_UN_CALIBRATE          ? "UN_CALIBRATE"    : \
    (x) == SDL_EVENT_CALIBRATE             ? "CALIBRATE"       : \
    (x) == SDL_EVENT_TRK_DISABLE           ? "TRK_DISABLE"     : \
    (x) == SDL_EVENT_TRK_ENABLE            ? "TRK_ENABLE"      : \
    (x) == SDL_EVENT_KEY_LEFT_ARROW        ? "KEY_LEFT_ARROW"  : \
    (x) == SDL_EVENT_KEY_RIGHT_ARROW       ? "KEY_RIGHT_ARROW" : \
    (x) == SDL_EVENT_KEY_UP_ARROW          ? "KEY_UP_ARROW"    : \
    (x) == SDL_EVENT_KEY_DOWN_ARROW        ? "KEY_DOWN_ARROW"  : \
    (x) == SDL_EVENT_KEY_SHIFT_LEFT_ARROW  ? "KEY_SHIFT_LEFT_ARROW"  : \
    (x) == SDL_EVENT_KEY_SHIFT_RIGHT_ARROW ? "KEY_SHIFT_RIGHT_ARROW" : \
    (x) == SDL_EVENT_KEY_SHIFT_UP_ARROW    ? "KEY_SHIFT_UP_ARROW"    : \
    (x) == SDL_EVENT_KEY_SHIFT_DOWN_ARROW  ? "KEY_SHIFT_DOWN_ARROW"  : \
                                             "????")
 
// AAA XXX 6 is tbd in the 4 following lines OR can just use one macro
#define AZDEG_TO_MSTEP(deg)    (rint((deg) * ((200. * 32. * 6.) / 360.)))
#define ELDEG_TO_MSTEP(deg)    (rint((deg) * ((200. * 32. * 6.) / 360.)))
#define MSTEP_TO_AZDEG(mstep)  ((mstep) * (360. / (200. * 32. * 6.)))
#define MSTEP_TO_ELDEG(mstep)  ((mstep) * (360. / (200. * 32. * 6.)))

#define MSTEP_360_DEG (200 * 32 * 6)
#define MSTEP_180_DEG (MSTEP_360_DEG / 2)
#define MSTEP_1_DEG   ((int)(MSTEP_360_DEG / 360. + .5))

#define MOTORS_CLOSED 0
#define MOTORS_OPEN   1
#define MOTORS_ERROR  2

#define MOTORS_STR(x) \
   ((x) == MOTORS_CLOSED ? "CLOSED" : \
    (x) == MOTORS_OPEN   ? "OPEN"   : \
    (x) == MOTORS_ERROR  ? "ERROR"  : \
                           "????")

#define CTLR_MOTOR_STATUS_VALID (microsec_timer() - ctlr_motor_status_us <= 1000000)

#define TEST_WITH_ONLY_AZ_MOTOR

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
double act_az, act_el;
bool   act_azel_available;
int    adj_az_mstep, adj_el_mstep;

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

void tele_debug_print_ctlr_motor_status(void);

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
        tele_debug_print_ctlr_motor_status();
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

    if (msg->id != MSGID_HEARTBEAT) {
        INFO("sending %s\n", MSGID_STR(msg->id));
    }

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
    bool tracking_enabled_last = false;
    bool connected_last = false;
    int  motors_last = MOTORS_CLOSED;
    bool calibrated_last = false;

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
                   strcmp(ctlr_motor_status.motor[0].error_status_str, "No_Err") == 0)
        {
            motors = MOTORS_OPEN;
#else
        } else if (ctlr_motor_status.motor[0].opened == 1 &&
                   strcmp(ctlr_motor_status.motor[0].operation_state_str, "NORMAL") == 0 &&
                   strcmp(ctlr_motor_status.motor[0].error_status_str, "No_Err") == 0) &&
                   ctlr_motor_status.motor[1].opened == 1 &&
                   strcmp(ctlr_motor_status.motor[1].operation_state_str, "NORMAL") == 0 &&
                   strcmp(ctlr_motor_status.motor[1].error_status_str, "No_Err") == 0
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

        // get tgt_az/el by calling sky routine
        sky_get_tgt_azel(&tgt_az, &tgt_el);

        // determine act_az/el from ctrl_motor_status shaft position
        if (calibrated) {
            int curr_az_mstep = ctlr_motor_status.motor[0].curr_pos_mstep;
            int curr_el_mstep __attribute__ ((unused)) = ctlr_motor_status.motor[1].curr_pos_mstep;
            act_az = MSTEP_TO_AZDEG(curr_az_mstep - cal_az0_mstep - adj_az_mstep);
#ifdef TEST_WITH_ONLY_AZ_MOTOR
            if (tracking_enabled) {
                act_el = tgt_el;
            }
#else
            act_el = MSTEP_TO_ELDEG(curr_el_mstep - cal_el0_mstep - adj_el_mstep);
#endif
            act_azel_available = true;
        } else {
            act_azel_available = false;
        }

        // AAA XXX invalid tgt and act  (send stop msg)

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
            int curr_el_mstep __attribute__ ((unused)) = ctlr_motor_status.motor[1].curr_pos_mstep;

            az_mstep = cal_az0_mstep + AZDEG_TO_MSTEP(tgt_az) + adj_az_mstep;
            el_mstep = cal_el0_mstep + AZDEG_TO_MSTEP(tgt_el) + adj_el_mstep;

            while (az_mstep - curr_az_mstep > MSTEP_180_DEG) az_mstep -= MSTEP_360_DEG;
            while (az_mstep - curr_az_mstep < -MSTEP_180_DEG) az_mstep += MSTEP_360_DEG;

            msg->id = MSGID_SET_POS_ALL;
            msg->datalen = sizeof(msg_set_pos_all_data_t);
            msg_set_pos_all_data->mstep[0] = az_mstep;
            msg_set_pos_all_data->mstep[1] = el_mstep;

            comm_send_msg(msg);

            time_last_set_pos_us = time_us;
        }

        // if tracking_enabled has changed to false stop the motors
        if (tracking_enabled_last && !tracking_enabled) {
            msg_t msg = { MSGID_STOP_ALL, 0 };
            comm_send_msg(&msg);
        }

        // debug print changes
        if (connected_last != connected ||
            motors_last != motors ||
            calibrated_last != calibrated ||
            tracking_enabled_last != tracking_enabled)
        {
            INFO("state changed to: connected=%d motors=%s calibrated=%d tracking_enabled=%d\n",
                 connected, 
                 MOTORS_STR(motors), 
                 calibrated, 
                 tracking_enabled);
        }

        // save last values
        connected_last = connected;
        motors_last = motors;
        calibrated_last = calibrated;
        tracking_enabled_last = tracking_enabled;

        // unlock mutex
        pthread_mutex_unlock(&mutex);
    }
}

void tele_ctrl_process_cmd(int event_id)
{
    // lock mutex
    pthread_mutex_lock(&mutex);

    INFO("processing event_id %s (0x%x)\n", EVENT_ID_STR(event_id), event_id);

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
            cal_el0_mstep = curr_el_mstep - AZDEG_TO_MSTEP(tgt_el);
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
                if (adj_az_mstep < -MSTEP_1_DEG) adj_az_mstep = -MSTEP_1_DEG;
                if (adj_az_mstep > MSTEP_1_DEG) adj_az_mstep = MSTEP_1_DEG;
            } else if (event_id == SDL_EVENT_KEY_UP_ARROW || event_id == SDL_EVENT_KEY_DOWN_ARROW) {
                adj_el_mstep += (event_id == SDL_EVENT_KEY_UP_ARROW ? 1 : -1);
                if (adj_el_mstep < -MSTEP_1_DEG) adj_el_mstep = -MSTEP_1_DEG;
                if (adj_el_mstep > MSTEP_1_DEG) adj_el_mstep = MSTEP_1_DEG;
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
                max_mstep = AZDEG_TO_MSTEP(5);
                mstep = (fine ? sign : sign * AZDEG_TO_MSTEP(0.5));
            } else {
                max_mstep = ELDEG_TO_MSTEP(5);
                mstep = (fine ? sign : sign * ELDEG_TO_MSTEP(0.5));
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
#if 0 // XXX AAA invalid azel
    } else if (invalid_azel) {
        sprintf(str1, "TGT %6.2f %6.2f INVALID_AZEL", tgt_az2, tgt_el);
#endif
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
        sprintf(str2, "ACT %6.2f %6.2f", act_az2, act_el);
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

// -----------------  TELE PANE HNDLR  ------------------------------------

int tele_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event)
{
    struct {
        int tbd;
    } * vars = pane_cx->vars;
    rect_t * pane = &pane_cx->pane;

    #define SDL_EVENT_MOUSE_MOTION   (SDL_EVENT_USER_DEFINED + 0)

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
        int fontsz = 20;
        char str1[100], str2[100], str3[100];

        // display status lines
        tele_ctrl_get_status(str1, str2, str3);
        sdl_render_printf(pane, COL2X(0,fontsz), ROW2Y(0,fontsz), fontsz, WHITE, BLACK, "%s", str1);
        sdl_render_printf(pane, COL2X(0,fontsz), ROW2Y(1,fontsz), fontsz, WHITE, BLACK, "%s", str2);
        sdl_render_printf(pane, COL2X(0,fontsz), pane->h-ROW2Y(1,fontsz), fontsz, WHITE, BLACK, "%s", str3);

        // register control events 
        sdl_render_text_and_register_event(
            pane, pane->w-COL2X(12,fontsz), ROW2Y(0,fontsz), fontsz, 
            motors != MOTORS_CLOSED ? "MOTORS_CLOSE" : "MOTORS_OPEN",
            LIGHT_BLUE, BLACK, 
            motors != MOTORS_CLOSED ? SDL_EVENT_MOTORS_CLOSE : SDL_EVENT_MOTORS_OPEN,
            SDL_EVENT_TYPE_MOUSE_CLICK, pane_cx);
        sdl_render_text_and_register_event(
            pane, pane->w-COL2X(12,fontsz), ROW2Y(1,fontsz), fontsz, 
            calibrated ? "UN_CALIBRATE" : "CALIBRATE",
            LIGHT_BLUE, BLACK, 
            calibrated ? SDL_EVENT_UN_CALIBRATE : SDL_EVENT_CALIBRATE,
            SDL_EVENT_TYPE_MOUSE_CLICK, pane_cx);
        sdl_render_text_and_register_event(
            pane, pane->w-COL2X(12,fontsz), ROW2Y(2,fontsz), fontsz, 
            tracking_enabled ? "TRK_DISABLE" : "TRK_ENABLE",
            LIGHT_BLUE, BLACK, 
            tracking_enabled ? SDL_EVENT_TRK_DISABLE : SDL_EVENT_TRK_ENABLE,
            SDL_EVENT_TYPE_MOUSE_CLICK, pane_cx);

        return PANE_HANDLER_RET_NO_ACTION;
    }

    // -----------------------
    // -------- EVENT --------
    // -----------------------

    if (request == PANE_HANDLER_REQ_EVENT) {
        int ret;

        // process the event_id
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

// -----------------  TELE DEBUG AND SUPPORT ROUTINES  --------------------

void tele_debug_print_ctlr_motor_status(void)
{
    DEBUG("opened                 %32lld %32lld\n", d->motor[0].opened, d->motor[1].opened);
    DEBUG("energized              %32lld %32lld\n", d->motor[0].energized, d->motor[1].energized);
    DEBUG("vin_voltage_mv         %32lld %32lld\n", d->motor[0].vin_voltage_mv, d->motor[1].vin_voltage_mv);
    DEBUG("curr_pos_mstep         %32lld %32lld\n", d->motor[0].curr_pos_mstep, d->motor[1].curr_pos_mstep);
    DEBUG("tgt_pos_mstep          %32lld %32lld\n", d->motor[0].tgt_pos_mstep, d->motor[1].tgt_pos_mstep);
    DEBUG("curr_vel_mstep_per_sec %32.2f %32.2f\n", d->motor[0].curr_vel_mstep_per_sec, d->motor[1].curr_vel_mstep_per_sec);
    DEBUG("operation_state_str    %32s %32s\n",     d->motor[0].operation_state_str, d->motor[1].operation_state_str);
    DEBUG("error_status_str       %32s %32s\n",     d->motor[0].error_status_str, d->motor[1].error_status_str);
}
