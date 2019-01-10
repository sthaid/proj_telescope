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

//
// typedefs
//

//
// variables
//

int sfd = -1;
bool connected;

//
// prototypes
//

void * tele_thread(void * cx);
void * heartbeat_thread(void * cx);
void process_recvd_msg(msg_t * msg);
void send_msg(msg_t * msg);

//
// design
//

#if 0
// -----------------------------------------------------------------------------------
// | CONNECTED CALIBRATED ENABLED                                            RESET   |  
// | TGT az el ACHIEVED                                                      CAL     |
// | ACT az el                                                               DISABLE |
// |                                                                                 |
// |                                                                                 |
// |                                                                                 |
// |                                                                                 |
// |                                  IMAGE                                          |
// |                                                                                 |
// |                                  IMAGE                                          |
// |                                                                                 |
// |                                  IMAGE                                          |
// |                                                                                 |
// |                                                                                 |
// |                                                                                 |
// |                                                                                 |
// |                                                                                 |
// | ADJ az el                                                                       |
// | CAL_AZ az motor_pos                                                             |
// | CAL_EL el motor_pos                                                             |
// -----------------------------------------------------------------------------------
//
// STATUS LINE 1
//   CONNECTED or CONNECTING
//   CALIBRATED or CAL_REQUIRED
//   ENABLED or DISABLED
//   
// STATUS LINE 2/3  (when calibrated)
//    TGT az el ACHIEVED|ACQUIRING|INVALID
//    ACT az el
//
// STATUS LINE 2/3  (when not calibrated)
//    MOTOR xxx xxx
//
// STATUS LINE 2/3  (when connecting)
//    ip_address
//
// XXX tbd - display camera status

Message Definitions

Routines
- tele_init
  . create the tele_thread
- tele_thread
  . connect to raspberrypi service
  . receive msgs from rpi
    . status msg
    . cam msg  TBD later
    . if tout then go to disconnected state,  the status msg should be rcvd 10 times per second
- tele_pane_hndlr
  . display
    - camera data
    - status / control text
  . controls
    - reset: send msg to rpi    CALL send_msg routine
    - cal: send msg to rpi
    - disable/enable: send msg to rpi
    - arrow keys, and shift arrow keys: send msg to rpi
      . arrow keys are .1 degrees, shift arrow is .01
      . these are handled by rpi based on whether or not calibrated

RPI
- service (msg thread)
    top: listen and accept connection
    while true
       recv msg with 100ms tout
       if msg rcvd then
          process msg
       endif
       send status msg,  maybe including result of the msg processed
       send camera msg whenever new image is available from the camera
    endwhile
    disconnect
    goto top
- camera thread
  . for now - just make a canned image available once per second

- process_recvd_msg()
  - reset: disable motor motion, clear calibration, stay connected
  - cal: save calibration data
  - disable/enable: set flag
- send_status_msg()
  - 
  - define the fields
#endif

// -----------------  TELE INIT  ------------------------------------------

int tele_init(char *incl_ss_obj_str)
{
    pthread_t thread_id;

    // create threads
    pthread_create(&thread_id, NULL, tele_thread, NULL);
    pthread_create(&thread_id, NULL, heartbeat_thread, NULL);

    // return success
    return 0;
}

// -----------------  THREADS  --------------------------------------------

void * tele_thread(void * cx)
{
    int rc, sfd_temp, len;
    struct sockaddr_in addr;
    struct timeval rcvto = {1, 0};  // sec, usec
    msg_t msg;

reconnect:
    // connect to tele_ctlr  (raspberry pi)
    sfd = socket(AF_INET, SOCK_STREAM, 0);
    if (getsockaddr(tele_ctlr, TELE_CTLR_PORT, &addr) < 0) {
        FATAL("failed to get address of %s\n", tele_ctlr);
    }
    do {
        rc = connect(sfd, (struct sockaddr *)&addr, sizeof(addr));
        if (rc == -1) {
            ERROR("failed connect to %s, %s\n", tele_ctlr, strerror(errno));
            sleep(1);
        }
    } while (rc == -1);
    connected = true;

    // set 1 second timeout for recv
    setsockopt(sfd, SOL_SOCKET, SO_RCVTIMEO, &rcvto, sizeof(rcvto));

    // receive msgs from tele_ctlr, and
    // process them
    while (true) {
        // recv msg  XXX data later
        // XXX or just call send and recv
        len = do_recv(sfd, &msg, sizeof(msg_t));
        if (len != sizeof(msg_t)) {
            ERROR("recvd msg with invalid len %d, %s\n", len, strerror(errno));
            break;
        }

        // process the recvd msg
        process_recvd_msg(&msg);
    }

    // lost connection; reconnect
    connected = false;
    sfd_temp = sfd;
    sfd = -1;
    close(sfd_temp);
    ERROR("lost connection to telescope, attempting to reconnect\n");
    goto reconnect;

    return NULL;
}

void process_recvd_msg(msg_t * msg)
{
    INFO("received %s\n", MSG_ID_STR(msg->id));
#if 0
    switch (msg->id) {
    case MSG_ID_RESET:
        break;
    case MSG_ID_CAL:
        break;
    case MSG_ID_ENABLE:
        break;
    case MSG_ID_DISABLE:
        break;
    default:
        break;
    }
#endif
}

void send_msg(msg_t * msg)
{
    int len;

    INFO("sending %s\n", MSG_ID_STR(msg->id));

    len = do_send(sfd, msg, sizeof(msg_t));
    if (len != sizeof(msg_t)) {
        ERROR("send failed, len=%d, %s\n", len, strerror(errno));
    }
}

void * heartbeat_thread(void * cx)
{
    msg_t msg;

    memset(&msg,0,sizeof(msg_t));
    msg.id = MSG_ID_HEARTBEAT;

    while (true) {
        if (connected) {
            send_msg(&msg);
        }
        usleep(200000);
    }
}

// -----------------  TELE PANE HNDLR  ------------------------------------

int tele_pane_hndlr(pane_cx_t * pane_cx, int request, void * init_params, sdl_event_t * event)
{
    struct {
        int tbd;
    } * vars = pane_cx->vars;
    rect_t * pane = &pane_cx->pane;

    #define SDL_EVENT_MOUSE_MOTION   (SDL_EVENT_USER_DEFINED + 0)
    // XXX #define SDL_EVENT_MOUSE_WHEEL    (SDL_EVENT_USER_DEFINED + 1)

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

        sdl_render_printf(pane, COL2X(0,fontsz), ROW2Y(0,fontsz), fontsz, WHITE, BLACK, "%s", "HELLO");

        // register control events 
        // XXX rect_t loc = {0,0,pane->w,pane->h};
        // XXX sdl_register_event(pane, &loc, SDL_EVENT_MOUSE_MOTION, SDL_EVENT_TYPE_MOUSE_MOTION, pane_cx);
        // XXX sdl_register_event(pane, &loc, SDL_EVENT_MOUSE_WHEEL, SDL_EVENT_TYPE_MOUSE_WHEEL, pane_cx);

        return PANE_HANDLER_RET_NO_ACTION;
    }

    // -----------------------
    // -------- EVENT --------
    // -----------------------

    if (request == PANE_HANDLER_REQ_EVENT) {
        switch (event->event_id) {
        case SDL_EVENT_MOUSE_MOTION:
        case SDL_EVENT_KEY_UP_ARROW:
        case SDL_EVENT_KEY_DOWN_ARROW:
        case SDL_EVENT_KEY_LEFT_ARROW:
        case SDL_EVENT_KEY_RIGHT_ARROW: {
            return PANE_HANDLER_RET_DISPLAY_REDRAW; }
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
