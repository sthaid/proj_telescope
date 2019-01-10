/*
Copyright (c) 2019 Steven Haid

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

void * ctlr_thread(void * cx);
void * heartbeat_thread(void * cx);
void process_recvd_msg(msg_t * msg);
void send_msg(msg_t * msg);
void * heartbeat_thread(void * cx);

// -----------------  MAIN  -----------------------------------------------

int main(int argc, char **argv)
{
    pthread_t thread_id;

    // create threads
    pthread_create(&thread_id, NULL, ctlr_thread, NULL);
    pthread_create(&thread_id, NULL, heartbeat_thread, NULL);

    // pause forever, the threads are doing the work
    while (true) {
        pause();
    }

    // return success
    return 0;
}

// -----------------  XXXX  -----------------------------------------------

void * ctlr_thread(void * cx)
{
    int listen_sfd, len, sfd_temp;
    struct sockaddr_in addr;
    socklen_t addrlen;
    char str[100];
    int reuseaddr = 1;
    struct timeval rcvto = {1, 0};  // sec, usec
    msg_t msg;

    // create listen socket
    listen_sfd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sfd == -1) {
        FATAL("socket listen_sfd\n");
    }

    // enable socket option to reuse the address
    if (setsockopt(listen_sfd, SOL_SOCKET, SO_REUSEADDR, &reuseaddr, sizeof(reuseaddr)) == -1) {
        FATAL("setsockopt SO_REUSEADDR, %s", strerror(errno));
    }

    // bind listen socket to any IP addr, and TELE_CTLR_PORT
    bzero(&addr,sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htons(INADDR_ANY);
    addr.sin_port = htons(TELE_CTLR_PORT);
    if (bind(listen_sfd, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
        FATAL("bind listen_sfd\n");
    }

    // wait for a connect request
    if (listen(listen_sfd, 0) == -1) {
        FATAL("listen\n");
    }

reconnect:
    // accept the connection
    addrlen = sizeof(addr);
    sfd = accept(listen_sfd, (struct sockaddr *)&addr, &addrlen);
    if (sfd == -1) {
        ERROR("accept, %s\n", strerror(errno));
    }
    INFO("accepted connection from %s\n",
         sock_addr_to_str(str, sizeof(str), (struct sockaddr*)&addr));
    connected = true;

    // set 1 second timeout for recv 
    if (setsockopt(sfd, SOL_SOCKET, SO_RCVTIMEO, &rcvto, sizeof(rcvto)) == -1) {
        FATAL("setsockopt SO_RCVTIMEO, %s", strerror(errno));
    }

    // send connected msg
    memset(&msg,0,sizeof(msg_t));
    msg.id = MSG_ID_CONNECTED;
    send_msg(&msg);

    // receive msgs from tele_ctlr, and
    // process them
    while (true) {
        // recv msg  XXX data later
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
    ERROR("lost connection from tcx, attempting to reconnect\n");
    goto reconnect;
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

