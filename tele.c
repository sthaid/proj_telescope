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

// XXX review
// XXX test plan
// XXX debug prints

#include "common.h"

//
// defines
//

#define SDL_EVENT_MOTORS_CLOSE    (SDL_EVENT_USER_DEFINED + 0)
#define SDL_EVENT_MOTORS_OPEN     (SDL_EVENT_USER_DEFINED + 1)
#define SDL_EVENT_UN_CALIBRATE    (SDL_EVENT_USER_DEFINED + 2)
#define SDL_EVENT_CALIBRATE       (SDL_EVENT_USER_DEFINED + 3)
#define SDL_EVENT_TRK_DISABLE     (SDL_EVENT_USER_DEFINED + 4)
#define SDL_EVENT_TRK_ENABLE      (SDL_EVENT_USER_DEFINED + 5)

#define EVENT_ID_STR(x) \
   ((x) == SDL_EVENT_MOTORS_CLOSE    ? "MOTORS_CLOSE"    : \
    (x) == SDL_EVENT_MOTORS_OPEN     ? "MOTORS_OPEN"     : \
    (x) == SDL_EVENT_UN_CALIBRATE    ? "UN_CALIBRATE"    : \
    (x) == SDL_EVENT_CALIBRATE       ? "CALIBRATE"       : \
    (x) == SDL_EVENT_TRK_DISABLE     ? "TRK_DISABLE"     : \
    (x) == SDL_EVENT_TRK_ENABLE      ? "TRK_ENABLE"      : \
    (x) == SDL_EVENT_KEY_LEFT_ARROW  ? "KEY_LEFT_ARROW"  : \
    (x) == SDL_EVENT_KEY_RIGHT_ARROW ? "KEY_RIGHT_ARROW" : \
    (x) == SDL_EVENT_KEY_UP_ARROW    ? "KEY_UP_ARROW"    : \
    (x) == SDL_EVENT_KEY_DOWN_ARROW  ? "KEY_DOWN_ARROW"  : \
                                       "????")
 
#define AZDEG_TO_MOTORSHAFTDEG 6.0  // XXX tbd need to measure
#define ELDEG_TO_MOTORSHAFTDEG 6.0  // XXX tbd

#define MOTORS_CLOSED 0
#define MOTORS_OPEN   1
#define MOTORS_ERROR  2

#define MOTORS_STR(x) \
   ((x) == MOTORS_CLOSED ? "CLOSED" : \
    (x) == MOTORS_OPEN   ? "OPEN"   : \
    (x) == MOTORS_ERROR  ? "ERROR"  : \
                           "????")

#define MAX_ADJ 1.0

#define CTLR_MOTOR_STATUS_VALID (microsec_timer() - ctlr_motor_status_us <= 1000000)

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
double cal_az0_motorshaft_deg;
double cal_el0_motorshaft_deg;
double tgt_az, tgt_el;
double act_az, act_el;
double adj_az, adj_el;
bool   act_azel_available;

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
            ERROR("failed connect to %s, %s\n", ctlr_ip, strerror(errno));
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
    if (len != sizeof(msg_t)) {
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
        } else if (ctlr_motor_status.motor[0].opened == 1 &&
                   strcmp(ctlr_motor_status.motor[0].operation_state_str, "NORMAL") == 0 &&
                   strcmp(ctlr_motor_status.motor[0].error_status_str, "No_Err") == 0)
#if 0  // XXX one motor
                   ctlr_motor_status.motor[1].opened == 1 &&
                   strcmp(ctlr_motor_status.motor[1].operation_state_str, "NORMAL") == 0 &&
                   strcmp(ctlr_motor_status.motor[1].error_status_str, "No_Err") == 0
#endif
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
            adj_az = adj_el = 0;
        }

        // get tgt_az/el by calling sky routine
        sky_get_tgt_azel(&tgt_az, &tgt_el);

        // determine act_az/el from ctrl_motor_status shaft position
        if (calibrated) {
            double curr_az_motorshaft_deg = ctlr_motor_status.motor[0].curr_pos_deg;
            double curr_el_motorshaft_deg = ctlr_motor_status.motor[1].curr_pos_deg;
            act_az = (curr_az_motorshaft_deg - cal_az0_motorshaft_deg) / AZDEG_TO_MOTORSHAFTDEG - adj_az;
            act_el = (curr_el_motorshaft_deg - cal_el0_motorshaft_deg) / ELDEG_TO_MOTORSHAFTDEG - adj_el;
            act_azel_available = true;
        } else {
            act_azel_available = false;
        }

        // XXX TBD
        // - home cmd
        // - invalid_tgt   send stop msg
        // - invalid_act

        // if tracking is enabled then set telescope position to tgt_az,tgt_el;
        // do this once per second
        if (tracking_enabled &&
            (time_us = microsec_timer()) >= time_last_set_pos_us + 1000000)
        {
            char msg_buffer[sizeof(msg_t)+sizeof(msg_set_pos_all_data_t)];
            msg_t *msg = (void*)msg_buffer;
            msg_set_pos_all_data_t *msg_set_pos_all_data = (void*)(msg+1);
            double az_motorshaft_deg, el_motorshaft_deg;

            az_motorshaft_deg = cal_az0_motorshaft_deg + (tgt_az + adj_az) * AZDEG_TO_MOTORSHAFTDEG;
            el_motorshaft_deg = cal_el0_motorshaft_deg + (tgt_el + adj_el) * ELDEG_TO_MOTORSHAFTDEG;
            // XXX pick shortest direction for az

            msg->id = MSGID_ADV_POS_SINGLE;
            msg->datalen = sizeof(msg_set_pos_all_data_t);
            msg_set_pos_all_data->deg[0] = az_motorshaft_deg;
            msg_set_pos_all_data->deg[1] = el_motorshaft_deg;

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

    INFO("processing event_id %s\n", EVENT_ID_STR(event_id));

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
            double curr_az_motorshaft_deg = ctlr_motor_status.motor[0].curr_pos_deg;
            double curr_el_motorshaft_deg = ctlr_motor_status.motor[1].curr_pos_deg;
            cal_az0_motorshaft_deg = curr_az_motorshaft_deg - tgt_az * AZDEG_TO_MOTORSHAFTDEG;
            cal_el0_motorshaft_deg = curr_el_motorshaft_deg - tgt_el * ELDEG_TO_MOTORSHAFTDEG;
            INFO("calibrated:    cal_motorshaft_deg curr_motorshaft_deg actual\n");
            INFO("calibrated: AZ %18f.2 %19.2f %6.2f\n",
                 cal_az0_motorshaft_deg, curr_az_motorshaft_deg, tgt_az);
            INFO("calibrated: EL %18f.2 %19.2f %6.2f\n",
                 cal_el0_motorshaft_deg, curr_el_motorshaft_deg, tgt_el);
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
    case SDL_EVENT_KEY_RIGHT_ARROW: {
        if (calibrated) {
            if (event_id == SDL_EVENT_KEY_LEFT_ARROW || event_id == SDL_EVENT_KEY_RIGHT_ARROW) {
                adj_az += (SDL_EVENT_KEY_LEFT_ARROW ? .01 : -.01);
                if (adj_az < -MAX_ADJ) adj_az = -MAX_ADJ;
                if (adj_az > MAX_ADJ) adj_az = MAX_ADJ;
            } else {
                adj_el += (SDL_EVENT_KEY_UP_ARROW ? .01 : -.01);
                if (adj_el < -MAX_ADJ) adj_el = -MAX_ADJ;
                if (adj_el > MAX_ADJ) adj_el = MAX_ADJ;
            }
        } else if (connected && motors == MOTORS_OPEN) {
            // XXX need fine control, maybe ALT ARROW
            char msg_buffer[sizeof(msg_t)+sizeof(msg_adv_pos_single_data_t)];
            msg_t *msg = (void*)msg_buffer;
            msg_adv_pos_single_data_t *msg_adv_pos_single_data = (void*)(msg+1);
            int h = (event_id == SDL_EVENT_KEY_LEFT_ARROW || event_id == SDL_EVENT_KEY_RIGHT_ARROW ? 0 : 1);
            double deg = (event_id == SDL_EVENT_KEY_UP_ARROW || event_id == SDL_EVENT_KEY_LEFT_ARROW) ? .5 : -.5;

            msg->id = MSGID_ADV_POS_SINGLE;
            msg->datalen = sizeof(msg_adv_pos_single_data_t);
            msg_adv_pos_single_data->h = h;
            msg_adv_pos_single_data->deg = deg;
            msg_adv_pos_single_data->max_deg = 5;

            comm_send_msg(msg);
        }
        break; }
    }

    // unlock mutex
    pthread_mutex_unlock(&mutex);
}

void tele_ctrl_get_status(char *str1, char *str2, char *str3)
{
    // lock mutex
    pthread_mutex_lock(&mutex);

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
#if 0 // XXX
    } else if (invalid_azel) {
        sprintf(str1, "TGT %6.2f %6.2f INVALID_AZEL", tgt_az, tgt_el);
#endif
    } else if (!tracking_enabled) {
        sprintf(str1, "TGT %6.2f %6.2f DISABLED", tgt_az, tgt_el);
    } else if (fabs(act_az-tgt_az) > .015 || fabs(act_el-tgt_el) > .015) {
        sprintf(str1, "TGT %6.2f %6.2f ACQUIRING", tgt_az, tgt_el);
    } else {
        sprintf(str1, "TGT %6.2f %6.2f ACQUIRED", tgt_az, tgt_el);
    }

    // STATUS LINE 2 - upper left
    //    ip_address      (when not connected)
    //    ACT az el       (when calibrated)
    //    MOTOR xxx xxx   (when not calibrated)
    if (!connected) {
        sprintf("IPADDR %s", ctlr_ip);
    } else if (act_azel_available) {
        sprintf(str2, "ACT %6.2f %6.2f", act_az, act_el);
    } else {
        char motor0_pos_str[32];
        char motor1_pos_str[32];
        if (CTLR_MOTOR_STATUS_VALID && ctlr_motor_status.motor[0].opened) {
            sprintf(motor0_pos_str, "%6.2f", ctlr_motor_status.motor[0].curr_pos_deg);
        } else {
            strcpy(motor0_pos_str, "  --  ");
        }
        if (CTLR_MOTOR_STATUS_VALID && ctlr_motor_status.motor[1].opened) {
            sprintf(motor1_pos_str, "%6.2f", ctlr_motor_status.motor[1].curr_pos_deg);
        } else {
            strcpy(motor1_pos_str, "  --  ");
        }
        sprintf(str2, "MOTOR %s %s", motor0_pos_str, motor1_pos_str);
    }

    // STATUS LINE 3 - lower left
    sprintf(str3, "ADJ %.2f %.2f", adj_az, adj_el);

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
        // XXX alt arrow keys too
        if (event->event_id == SDL_EVENT_KEY_UP_ARROW ||
            event->event_id == SDL_EVENT_KEY_DOWN_ARROW ||
            event->event_id == SDL_EVENT_KEY_LEFT_ARROW ||
            event->event_id == SDL_EVENT_KEY_RIGHT_ARROW)
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
    DEBUG("opened               %32lld %32lld\n", d->motor[0].opened, d->motor[1].opened);
    DEBUG("energized            %32lld %32lld\n", d->motor[0].energized, d->motor[1].energized);
    DEBUG("vin_voltage_v        %32.2f %32.2f\n", d->motor[0].vin_voltage_v, d->motor[1].vin_voltage_v);
    DEBUG("curr_pos_deg         %32.2f %32.2f\n", d->motor[0].curr_pos_deg, d->motor[1].curr_pos_deg);
    DEBUG("tgt_pos_deg          %32.2f %32.2f\n", d->motor[0].tgt_pos_deg, d->motor[1].tgt_pos_deg);
    DEBUG("curr_vel_degpersec   %32.2f %32.2f\n", d->motor[0].curr_vel_degpersec, d->motor[1].curr_vel_degpersec);
    DEBUG("operation_state_str  %32s %32s\n",     d->motor[0].operation_state_str, d->motor[1].operation_state_str);
    DEBUG("error_status_str     %32s %32s\n",     d->motor[0].error_status_str, d->motor[1].error_status_str);
}
