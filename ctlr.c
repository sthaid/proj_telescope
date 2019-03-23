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

#if 0  
On Raspberry Pi, add the following to /etc/rc.local to automatically
start the ctlr program when the Raspberry Pi boots.
    # Start telescope ctlr
    if [ -x /home/haid/proj_telescope/ctlr ]
    then
      su haid -c "cd /home/haid/proj_telescope; LD_LIBRARY_PATH=/usr/local/lib ./ctlr </dev/null &>>ctlr.log &"
    fi  
    # disable wifi power mgmt
    iwconfig wlan0 power off

Add the following to /etc/sysctl.conf, and apply using 'sysctl -p'
    net.core.wmem_max = 8000000
    net.core.rmem_max = 8000000

Create the following udev rules. And apply the udev rules by rebooting or 
run 'sudo udevadm control --reload-rules && udevadm trigger'.
    /etc/udev/rules.d/99-pololu.rules
       SUBSYSTEM=="usb", ATTRS{idVendor}=="1ffb", MODE="0666"
    /etc/udev/rules.d/10-local.rules
       KERNEL=="video*", MODE="0666"

For more info refer to devel/NOTES.raspberrypi.
#endif

// This program should work on either 32 or 64 bit linux. Raspbian is 32 bit.
// Guidelines / Notes:
// - don't use 'long'
// - size_t is 4 bytes on Raspian and 8 bytes on 64 bit Linux. 
//   Use %zd when printing a size_t
// - common.h enforces not using 'long'
// - fortunately tic.h doesn't use 'long'

// Logitech QuickCam Pro 4000
// - v4l2-ctl --list-formats-ext
//         Index       : 1
//         Type        : Video Capture
//         Pixel Format: 'YU12'
//         Name        : Planar YUV 4:2:0
// - http://wiki.oz9aec.net/index.php/Pixel_formats 
//   says YU12 is 1420
// - http://fourcc.org/yuv.php#IYUV
//   says IYUV and I420 are identical
// - SDL supports
//   SDL_PIXELFORMAT_IYUV

#include "common.h"
#include <tic.h>

//
// defines
//

//#define MOTOR_UNIT_TEST

#define SWAP32(x) ((((x) >> 0) & 0xff) << 24 | \
                   (((x) >> 8) & 0xff) << 16 | \
                   (((x) >>16) & 0xff) <<  8 | \
                   (((x) >>24) & 0xff) <<  0)

#define MULTI_CHAR(a,b,c,d) ( ((a)<<0) | ((b)<<8) | ((c)<<16) | ((d)<<24) )

#define FMT_MJPG  MULTI_CHAR('M','J','P','G')
#define FMT_YU12  MULTI_CHAR('Y','U','1','2')

//
// typedefs
//

//
// variables
//

static int cam_img_receipt_id;
static int cam_img_lastsnd_id;

static msg_calibration_values_data_t tcx_cal_values;

//
// prototypes
//

// message routines
static int comm_init(void);
static void comm_send_msg(msg_t * msg);
static void comm_set_sock_opts(void);
static void comm_verify_sock_opts(void);

// motor routines
static int motor_init(void);
static void motor_exit(void);
static int motor_open_all(void);
static void motor_close_all(void);
static int motor_open(int h);
static void motor_close(int h);
static int motor_set_pos(int h, int mstep);  
static int motor_adv_pos(int h, int mstep, int max_mstep);   
static int motor_request_stop(int h);
static int motor_wait_for_stopped(int h);
static int motor_request_all_stop(void); 
static int motor_wait_for_all_stopped(void); 
#ifdef MOTOR_UNIT_TEST
static void motor_unit_test(void);
#endif

// camera routines
static int cam_init(void);
static int cam_reset(void);

// -----------------  MAIN  -----------------------------------------------

int main(int argc, char **argv)
{
    int rc;

    // this performs sanity checks for minilzo
    compress_init();

    // motor initialize
    rc = motor_init();
    if (rc < 0) {
        ERROR("motor_init failed\n"); // XXX should be FATAL
    }

    // camera initialize
    rc = cam_init();
    if (rc < 0) {
        FATAL("cam_init failed\n");
    }

    // comm initialize
    rc = comm_init();
    if (rc < 0) {
        FATAL("comm_init failed\n");
    }

#ifdef MOTOR_UNIT_TEST
    // unit_test
    motor_unit_test();
#endif

    // pause 
    while (true) pause();
    return 0;
}

// -----------------  COMM  -----------------------------------------------

//
// variables
//

static int sfd = -1;
static bool connected;

//
// prototypes
//

static void * comm_thread(void * cx);
static int comm_process_recvd_msg(msg_t * msg);

//
// code
//

static int comm_init(void)
{
    pthread_t thread_id;

    pthread_create(&thread_id, NULL, comm_thread, NULL);

    return 0;
}

static void * comm_thread(void * cx)
{
    int listen_sfd, len, sfd_temp, rc;
    struct sockaddr_in addr;
    socklen_t addrlen;
    char str[100];
    int reuseaddr = 1;
    char msg_buffer[1000];
    msg_t * msg = (msg_t*)msg_buffer; 

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
    connected = true;

    // if we have calibration data then send it
    if (tcx_cal_values.cal_valid) {
        INFO("sending calibration values\n");
        msg->id = MSGID_CALIBRATION_VALUES;
        msg->data_len = sizeof(msg_calibration_values_data_t);
        memcpy(msg->data, &tcx_cal_values, sizeof(msg_calibration_values_data_t));
        comm_send_msg(msg);
    }

    // receive msgs from tele_ctlr, and
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

    sfd_temp = sfd;
    sfd = -1;
    close(sfd_temp);

    motor_request_all_stop();
    motor_wait_for_all_stopped();

    ERROR("lost connection from tcx, attempting to reconnect\n");
    goto reconnect;
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

    DEBUG("received %s\n", MSGID_STR(msg->id));

    switch (msg->id) {
    case MSGID_OPEN_ALL: {
        CHECK_DATALEN(0);
        motor_open_all();
        break; }
    case MSGID_CLOSE_ALL: {
        CHECK_DATALEN(0);
        motor_close_all();
        break; }
    case MSGID_STOP_ALL: {
        CHECK_DATALEN(0);
        motor_request_all_stop();
        break; }
    case MSGID_ADV_POS_SINGLE: {
        CHECK_DATALEN(sizeof(msg_adv_pos_single_data_t));
        msg_adv_pos_single_data_t * d = (void*)msg->data;
        motor_adv_pos(d->h, d->mstep, d->max_mstep);
        break; }
    case MSGID_SET_POS_ALL: {
        CHECK_DATALEN(sizeof(msg_set_pos_all_data_t));
        msg_set_pos_all_data_t * d = (void*)msg->data;
        int h;
        for (h = 0; h < MAX_MOTOR; h++) {
            motor_set_pos(h, d->mstep[h]);
        }
        break; }
    case MSGID_SHUTDN_CTLR: {
        CHECK_DATALEN(0);
        motor_close_all();
        INFO("shutting down ctlr\n");
        system("sudo shutdown -P now");        
        exit(0);
        break; }
    case MSGID_CAM_CTRLS_INCR_DECR: {
        msg_cam_ctrls_incr_decr_t * d = (void*)msg->data;
        char msg_reply_buff[100];
        msg_t *msg_reply = (void*)msg_reply_buff;
        msg_cam_ctrls_get_t *msg_reply_data = (void*)msg_reply->data;
        int cid = d->cid, value;

        // perform the requested incr or decr
        CHECK_DATALEN(sizeof(msg_cam_ctrls_incr_decr_t));
        if (cam_ctrls_incr_decr(d->cid, d->incr_flag) != 0) {
            break;
        }

        // get the updated value, and send it to tcx
        cam_ctrls_get(cid, &value);
        msg_reply->id         = MSGID_CAM_CTRLS_GET;
        msg_reply->data_len   = sizeof(msg_cam_ctrls_get_t);
        msg_reply_data->cid   = cid;
        msg_reply_data->value = value;
        comm_send_msg(msg_reply);
        break; }
    case MSGID_CAM_CTRLS_RESET:
        cam_ctrls_set_all_to_default();
        break;
    case MSGID_CAM_CTRLS_SET: {
        msg_cam_ctrls_set_t * d = (void*)msg->data;
        cam_ctrls_set(d->cid, d->value);
        break; }
    case MSGID_CAM_RESET_REQ:
        cam_reset();
        break;
    case MSGID_CAM_IMG_ACK_RECEIPT: {
        msg_cam_img_ack_receipt_t * d = (void*)msg->data;
        CHECK_DATALEN(sizeof(msg_cam_img_ack_receipt_t));
        cam_img_receipt_id = d->img_id;
        break; }
    case MSGID_CALIBRATION_VALUES: {
        msg_calibration_values_data_t * d = (void*)msg->data;
        CHECK_DATALEN(sizeof(msg_calibration_values_data_t));
        INFO("received calibration values, valid=%d\n", d->cal_valid);
        tcx_cal_values = *d;
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
    len = do_send(sfd, msg, sizeof(msg_t) + msg->data_len);
    pthread_mutex_unlock(&send_mutex);

    if (len != sizeof(msg_t) + msg->data_len) {
        ERROR("send failed, len=%d, %s\n", len, strerror(errno));
    }
}

static void comm_set_sock_opts(void)
{
    int rc, optval;

    if (sfd == -1) {
        FATAL("BUG: sfd is -1\n");
    }

    // set send-buffer size 
    optval = SOCK_BUFF_SIZE;
    rc = setsockopt(sfd, SOL_SOCKET, SO_SNDBUF, &optval, sizeof(optval));
    if (rc == -1) {
        FATAL("setsockopt SO_SNDBUF, %s", strerror(errno));
    }

    // set rcv-buffer size
    optval = SOCK_BUFF_SIZE;
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
    if (optval != 2*SOCK_BUFF_SIZE) {
        INFO("add to /etc/sysctl.conf:\n");
        INFO("   net.core.wmem_max = 8000000\n");
        INFO("   net.core.rmem_max = 8000000\n");
        FATAL("getsockopt SO_SNDBUF, optval=%d\n", optval);
    }

    // verify recv-buffer size
    optval = 0;
    optlen = sizeof(optval);
    rc = getsockopt(sfd, SOL_SOCKET, SO_RCVBUF, &optval, &optlen);       
    if (rc == -1) {
        FATAL("getsockopt SO_RCVBUF, %s", strerror(errno));
    }
    if (optval != 2*SOCK_BUFF_SIZE) {
        INFO("add to /etc/sysctl.conf:\n");
        INFO("   net.core.wmem_max = 8000000\n");
        INFO("   net.core.rmem_max = 8000000\n");
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

// -----------------  MOTOR  ----------------------------------------------

// XXX more motor err chks

//
// defines
//

#define VOLTAGE_OK(mv) (mv >= 10000 && mv <= 15000)

#define OPERATION_STATE(v)    (tic_variables_get_operation_state(v))
#define ENERGIZED(v)          (tic_variables_get_energized(v))
#define POSITION_UNCERTAIN(v) (tic_variables_get_position_uncertain(v))
#define ERROR_STATUS(v)       (tic_variables_get_error_status(v))
#define ERRORS_OCCURRED(v)    (tic_variables_get_errors_occurred(v))
#define VIN_VOLTAGE(v)        (tic_variables_get_vin_voltage(v))
#define TARGET_POSITION(v)    (tic_variables_get_target_position(v))
#define CURRENT_POSITION(v)   (tic_variables_get_current_position(v))
#define TARGET_VELOCITY(v)    (tic_variables_get_target_velocity(v))
#define CURRENT_VELOCITY(v)   (tic_variables_get_current_velocity(v))

#define MAX_SPEED (9.0)  // deg/sec
#define MAX_ACCEL (5.4)  // deg/sec/sec

#define DEG2MICROSTEP(d)                (rint((d) * ((200.*32.) / 360.)))
#define MICROSTEP2DEG(mstep)            ((mstep) * (360. / (200.*32.)))
#define MICROSTEPVEL2DEGPERSEC(mstepv)  ((mstepv) * (360. / (200.*32.)) / 10000.)

#define AZ_MOTOR_SN "00247330"
#define EL_MOTOR_SN "00250237"

//
// variables
//

static struct motor_s {
    char serial_number[50];
    tic_handle *tic_handle;
} motor[MAX_MOTOR];

static pthread_mutex_t motor_mutex = PTHREAD_MUTEX_INITIALIZER;

static tic_device ** motor_devices;
static size_t max_motor_devices;

static bool motor_initialized;
static bool motor_keepalive_thread_running;
static bool motor_getstatus_thread_running;

#ifdef MOTOR_UNIT_TEST
static FILE *fp_unit_test[MAX_MOTOR];
#endif

//
// prototypes
//

static void * motor_getstatus_thread(void * cx);
static void * motor_keepalive_thread(void * cx);
static int motor_check_settings(int h, bool verbose);
static void motor_inspect_variables(int h, bool verbose);
static char * motor_operation_state_str(int op_state);
static char * motor_error_status_str(int err_stat);

// ---- init & exit ----

// intended to be called once at startup
static int motor_init(void)
{
    tic_error * err;
    pthread_t thread_id;
    int h;

    // fatal error if already initialized, and 
    // set initialized flag
    if (motor_initialized) {
        FATAL("already initialized\n");
    }
    motor_initialized = true;

    // register motor exit handler
    atexit(motor_exit);

    // get list of connected devices;
    err = tic_list_connected_devices(&motor_devices, &max_motor_devices);
    if (err) {
        ERROR("tic_list_connected_devices, %s\n", tic_error_get_message(err));
        return -1;
    }

    // sanity check that there are not more motors connected than this program is
    // configured to support
    if (max_motor_devices > MAX_MOTOR) {
        ERROR("too many motors, %zd\n", max_motor_devices);
        return -1;
    }

    // verify both AZ and EL motors are present, and
    // swap them if needed so that h=0 is AZ and H=1 is EL
    if (max_motor_devices != 2) {
        ERROR("max_motor_devices=%zd must be 2\n", max_motor_devices);
        return -1;
    }
    const char * sn0 = tic_device_get_serial_number(motor_devices[0]);
    const char * sn1 = tic_device_get_serial_number(motor_devices[1]);
    if (strcmp(sn0, AZ_MOTOR_SN) != 0 && strcmp(sn0, EL_MOTOR_SN) != 0) {
        ERROR("sn0='%s' is not AZ_MOTOR_SN or EL_MOTOR_SN\n", sn0);
        return -1;
    }
    if (strcmp(sn1, AZ_MOTOR_SN) != 0 && strcmp(sn1, EL_MOTOR_SN) != 0) {
        ERROR("sn1='%s' is not AZ_MOTOR_SN or EL_MOTOR_SN\n", sn1);
        return -1;
    }
    if (strcmp(sn0, AZ_MOTOR_SN) != 0) {
        SWAP_VALUES(motor_devices[0], motor_devices[1]);
    }

    // print list of connected devices
    INFO("number of devices found = %zd\n", max_motor_devices);
    for (h = 0; h < max_motor_devices; h++) {
        const char * sn = tic_device_get_serial_number(motor_devices[h]);
        INFO("device %d: serial_number=%s %s\n", 
             h, sn, 
             (strcmp(sn, AZ_MOTOR_SN) == 0 ? "AZ_MOTOR" :
              strcmp(sn, EL_MOTOR_SN) == 0 ? "EL_MOTOR" :
                                             "????"));
    }

#ifdef MOTOR_UNIT_TEST
    // open unit test log files
    for (h = 0; h < max_motor_devices; h++) {
        char fn[100];
        sprintf(fn, "ctlr_unit_test_%d.log", h);
        fp_unit_test[h] = fopen(fn, "a");
        if (fp_unit_test[h] == NULL) {
            FATAL("failed to open %s, %s\n", fn, strerror(errno));
        }
        setlinebuf(fp_unit_test[h]);
    }
#endif

    // create motor threads
    pthread_create(&thread_id, NULL, motor_keepalive_thread, NULL);
    pthread_create(&thread_id, NULL, motor_getstatus_thread, NULL);
    while (!motor_keepalive_thread_running || !motor_getstatus_thread_running) {
        usleep(1000);
    }

    // return success
    return 0;
}

// intended to be called once when program is exitting
static void motor_exit(void)
{
    int i;

    // it not initialized just return
    if (!motor_initialized) {
        return;
    }

    // close all motors, this will deenergize them
    motor_close_all();

    // clear initialized flag
    motor_initialized = false;

    // wait for threads to terminate,
    // the threads terminate as a result of motor_initialized being false
    while (motor_keepalive_thread_running || motor_getstatus_thread_running) {
        usleep(1000);
    }

    // cleanup
    for (i = 0; i < max_motor_devices; i++) {
        tic_device_free(motor_devices[i]);
    }
    tic_list_free(motor_devices);
}

// ---- open and close all motors ----

static int motor_open_all(void)
{
    int h;

    INFO("starting\n");

    // call motor_open for all motors that have been detected
    for (h = 0; h < max_motor_devices; h++) {
        if (motor_open(h) < 0) {
            return -1;
        }
    }

    INFO("done\n");
    return 0;
}

static void motor_close_all(void)
{
    int h;

    INFO("starting\n");

    // stop all motors
    motor_request_all_stop();
    motor_wait_for_all_stopped();

    // call motor_close for all motors that are open
    for (h = 0; h < max_motor_devices; h++) {
        if (motor[h].tic_handle == NULL) {
            continue;
        }
        motor_close(h);
    }

    INFO("done\n");
}

// ---- open and close single motor ----

static int motor_open(int h)
{
    tic_error * err;
    tic_handle * tic_handle;
    int rc;

    INFO("called for motor %d\n", h);

    // check that motor[h] is closed
    if (h >= MAX_MOTOR || motor[h].tic_handle != NULL) {
        ERROR("invalid handle %d\n", h);
        return -1;
    }

    // acquire mutex
    pthread_mutex_lock(&motor_mutex);

    // open handle to the tic device in list[i]
    err = tic_handle_open(motor_devices[h], &tic_handle);
    if (err) {
        ERROR("tic_handle_open %d, %s\n", h, tic_error_get_message(err));
        pthread_mutex_unlock(&motor_mutex);
        return -1;
    }

    // save motor info
    motor[h].tic_handle = tic_handle;
    strcpy(motor[h].serial_number, tic_device_get_serial_number(motor_devices[h]));

    // reset the tic
    tic_reset(tic_handle);
    tic_set_target_position(tic_handle, 0);

    // check that the motor is configured correctly;
    // (use tic_gui or ticcmd to make corrections)
    rc = motor_check_settings(h, false);
    if (rc != 0) {        
        pthread_mutex_unlock(&motor_mutex);
        return -1;
    }
    motor_inspect_variables(h, false);

    // energize and exit_safe_start 
    tic_energize(tic_handle);
    tic_exit_safe_start(tic_handle);

    // release mutex
    pthread_mutex_unlock(&motor_mutex);

    // return success
    return 0;
}

static void motor_close(int h)
{
    INFO("called for motor %d\n", h);

    // check that motor[h] has been opened
    if (h >= MAX_MOTOR || motor[h].tic_handle == NULL) {
        ERROR("invalid handle %d\n", h);
        return;
    }

    // stop the motor
    motor_request_stop(h);
    motor_wait_for_stopped(h);

    // acquire mutex
    pthread_mutex_lock(&motor_mutex);

    // enter_safe_start,
    // deenergize, and 
    // close tic_handle
    tic_enter_safe_start(motor[h].tic_handle);
    tic_deenergize(motor[h].tic_handle);
    tic_handle_close(motor[h].tic_handle);

    // clear motor_s that was just closed
    memset(&motor[h], 0, sizeof(struct motor_s));  

    // release mutex
    pthread_mutex_unlock(&motor_mutex);
}

// ---- set positions ----

static int motor_set_pos(int h, int mstep)
{
    tic_error * err;

    // check that motor[h] has been opened
    if (h >= MAX_MOTOR || motor[h].tic_handle == NULL) {
        ERROR("invalid handle %d\n", h);
        return -1;
    }

    // set motor position
    err = tic_set_target_position(motor[h].tic_handle, mstep);
    if (err) {
        ERROR("tic_set_target_position %d, %s\n", h, tic_error_get_message(err));
        return -1;
    }

    // return success
    return 0;
}

static int motor_adv_pos(int h, int mstep, int max_mstep)
{
    int curr_pos, tgt_pos;
    tic_error * err;
    tic_variables * v;

    // check that motor[h] has been opened
    if (h >= MAX_MOTOR || motor[h].tic_handle == NULL) {
        ERROR("invalid handle %d\n", h);
        return -1;
    }

    // get current and target position
    tic_get_variables(motor[h].tic_handle, &v, false);
    curr_pos = CURRENT_POSITION(v);
    tgt_pos = TARGET_POSITION(v);
    tic_variables_free(v);

    // ignore request if tgt_pos is already more than max_mstep from curr_pos
    if (mstep > 0 && tgt_pos - curr_pos > max_mstep) {
        return 0;
    }
    if (mstep < 0 && tgt_pos - curr_pos < -max_mstep) {
        return 0;
    }
    if (mstep == 0) {
        return 0;
    }

    // set new target position by adding to the current tgt_pos
    err = tic_set_target_position(motor[h].tic_handle, tgt_pos+mstep);
    if (err) {
        ERROR("tic_set_target_position %d, %s\n", h, tic_error_get_message(err));
        return -1;
    }

    // success
    return 0;
}

// ---- stop motors ----

static int motor_request_stop(int h)
{
    tic_variables * v;
    int stop_pos_mstep;
    double curr_pos_mstep, curr_vel_mstep_per_sec;

    // check that motor[h] has been opened
    if (h >= MAX_MOTOR || motor[h].tic_handle == NULL) {
        ERROR("invalid handle %d\n", h);
        return -1;
    }

    // get curr_pos and curr_vel
    tic_get_variables(motor[h].tic_handle, &v, false);
    curr_pos_mstep = CURRENT_POSITION(v);
    curr_vel_mstep_per_sec = CURRENT_VELOCITY(v) / 10000.0;
    tic_variables_free(v);

    // determine pos to stop at based on max decel
    if (curr_vel_mstep_per_sec >= 0) {
        stop_pos_mstep = curr_pos_mstep + 
                         (curr_vel_mstep_per_sec * curr_vel_mstep_per_sec / (2. * DEG2MICROSTEP(MAX_ACCEL)));
    } else {
        stop_pos_mstep = curr_pos_mstep - 
                         (curr_vel_mstep_per_sec * curr_vel_mstep_per_sec / (2. * DEG2MICROSTEP(MAX_ACCEL)));
    }

    // set pos
    motor_set_pos(h, stop_pos_mstep);

    // success
    return 0;
}

static int motor_wait_for_stopped(int h)
{
    #define POLL_INTVL_US 100000
    tic_variables * v;
    int curr_vel;
    int duration_us = 0;

    // check that motor[h] has been opened
    if (h >= MAX_MOTOR || motor[h].tic_handle == NULL) {
        ERROR("invalid handle %d\n", h);
        return -1;
    }

    // loop until velocity is 0, or 10 sec timeout
    while (true) {
        tic_get_variables(motor[h].tic_handle, &v, false);
        curr_vel = CURRENT_VELOCITY(v);
        tic_variables_free(v);

        if (curr_vel == 0) {
            DEBUG("motor %d is stopped\n", h);
            return 0;
        }
        if (duration_us > 10000000) {  // 10 secs
            ERROR("timedout waiting for motor %d to stop\n", h);
            return -1;
        }

        usleep(POLL_INTVL_US);
        duration_us += POLL_INTVL_US;
    } while (curr_vel);
}

static int motor_request_all_stop(void)
{
    int h;

    INFO("request all motors stop\n");
    for (h = 0; h < max_motor_devices; h++) {
        if (motor[h].tic_handle == NULL) {
            continue;
        }
        motor_request_stop(h);
    }

    return 0;
}

static int motor_wait_for_all_stopped(void)
{
    int h;

    INFO("wait for all motors to be stopped\n");
    for (h = 0; h < max_motor_devices; h++) {
        if (motor[h].tic_handle == NULL) {
            continue;
        }
        motor_wait_for_stopped(h);
    }

    INFO("done\n");
    return 0;
}

// ---- motor_getstatus_thread ----

static void * motor_getstatus_thread(void * cx)
{
    int h;
    tic_variables * variables[MAX_MOTOR];
    char msg_motor_status_buff[1000];
    msg_t * msg_motor_status = (void*)msg_motor_status_buff;
    msg_motor_status_data_t * msg_motor_status_data = (msg_motor_status_data_t*)msg_motor_status->data;

    // init
    memset(variables, 0, sizeof(variables));
    motor_getstatus_thread_running = true;

    while (motor_initialized) {
        // acquire mutex
        pthread_mutex_lock(&motor_mutex);

        // for each open motor get variables
        for (h = 0; h < MAX_MOTOR; h++) {
            if (motor[h].tic_handle == NULL) {
                continue;
            }

            // get the variables
            tic_get_variables(motor[h].tic_handle, &variables[h], true);
            // XXX this can this fail 

            // also check for need to energize the motor because the voltage is now okay
            tic_variables *v = variables[h];
            if (ENERGIZED(v) == 0 && VOLTAGE_OK(VIN_VOLTAGE(v))) {
                INFO("energizing motor %d\n", h);
                tic_energize(motor[h].tic_handle);
                tic_exit_safe_start(motor[h].tic_handle);
            }
        }

        // release mutex
        pthread_mutex_unlock(&motor_mutex);

#ifdef MOTOR_UNIT_TEST
        // unit test, print motor variables to a seperate log file for each motor;
        // print every 4th time, which is once per sec
        static int count;
        if ((count++ % 4) == 0) {
            for (h = 0; h < MAX_MOTOR; h++) {
                tic_variables *v = variables[h];
                char time_str[100];

                if (motor[h].tic_handle == NULL) {
                    continue;
                }

#if 0
                fprintf(fp_unit_test[h], "%s %8.2f %8.2f %8.2f %8.2f %d %4.1f %s %s\n",
                        time2str(time_str, get_real_time_us(), false, true, true),
                        MICROSTEP2DEG(CURRENT_POSITION(v)),
                        MICROSTEP2DEG(TARGET_POSITION(v)),
                        MICROSTEPVEL2DEGPERSEC(CURRENT_VELOCITY(v)),
                        MICROSTEPVEL2DEGPERSEC(TARGET_VELOCITY(v)),
                        ENERGIZED(v),
                        VIN_VOLTAGE(v)/1000.,
                        motor_operation_state_str(OPERATION_STATE(v)),
                        motor_error_status_str(ERROR_STATUS(v)));
#else
                fprintf(fp_unit_test[h], "%s %8d %8d %8d %8d %8d %8d %s %s\n",
                        time2str(time_str, get_real_time_us(), false, true, true),
                        CURRENT_POSITION(v),
                        TARGET_POSITION(v),
                        CURRENT_VELOCITY(v),
                        TARGET_VELOCITY(v),
                        ENERGIZED(v),
                        VIN_VOLTAGE(v),
                        motor_operation_state_str(OPERATION_STATE(v)),
                        motor_error_status_str(ERROR_STATUS(v)));
#endif
            } 
        }
#endif

        // fill in the status msg, and send it
        msg_motor_status->id = MSGID_MOTOR_STATUS;
        msg_motor_status->data_len = sizeof(msg_motor_status_data_t);
        memset(msg_motor_status_data, 0, sizeof(msg_motor_status_data_t));
        for (h = 0; h < MAX_MOTOR; h++) {
            tic_variables *v = variables[h];

            if (motor[h].tic_handle == NULL) {
                continue;
            }

            msg_motor_status_data->motor[h].opened                 = 1;
            msg_motor_status_data->motor[h].energized              = ENERGIZED(v);
            msg_motor_status_data->motor[h].vin_voltage_mv         = VIN_VOLTAGE(v);
            msg_motor_status_data->motor[h].curr_pos_mstep         = CURRENT_POSITION(v);
            msg_motor_status_data->motor[h].tgt_pos_mstep          = TARGET_POSITION(v);
            msg_motor_status_data->motor[h].curr_vel_mstep_per_sec = CURRENT_VELOCITY(v) / 10000.0;
            msg_motor_status_data->motor[h].spare1                 = 0;
            strncpy(msg_motor_status_data->motor[h].operation_state_str, 
                    motor_operation_state_str(OPERATION_STATE(v)),
                    sizeof(msg_motor_status_data->motor[h].operation_state_str)-1);
            strncpy(msg_motor_status_data->motor[h].error_status_str,
                    motor_error_status_str(ERROR_STATUS(v)),
                    sizeof(msg_motor_status_data->motor[h].error_status_str)-1);
        }
        comm_send_msg(msg_motor_status);

        // if the motor status is not okay, for example this could be due to 
        // low motor voltage, then clear the saved tcx calibration
        if (!MOTOR_STATUS_OKAY(msg_motor_status_data->motor)) {
            if (tcx_cal_values.cal_valid) {
                INFO("clearing tcx_cal_values.cal_valid because !MOTOR_STATUS_OKAY\n");
            }
            tcx_cal_values.cal_valid = 0;
        }
            
        // free variables
        for (h = 0; h < MAX_MOTOR; h++) {
            if (variables[h] != NULL) {
                tic_variables_free(variables[h]);
                variables[h] = NULL;
            }
        }

        // delay 0.25 sec
        usleep(250000);
    }

    // terminate
    motor_getstatus_thread_running = false;
    return NULL;
}

// ---- motor_keepalive_thread ----

static void * motor_keepalive_thread(void * cx)
{
    int h;

    motor_keepalive_thread_running = true;

    while (motor_initialized) {
        // acquire mutex
        pthread_mutex_lock(&motor_mutex);

        // for each open motor reset the command timeout
        for (h = 0; h < MAX_MOTOR; h++) {
            if (motor[h].tic_handle == NULL) {
                continue;
            }
            tic_reset_command_timeout(motor[h].tic_handle);
        }

        // release mutex
        pthread_mutex_unlock(&motor_mutex);

        // delay .2 sec
        usleep(200000);
    }

    motor_keepalive_thread_running = false;

    return NULL;
}

// ---- misc motor support routines ----

static int motor_check_settings(int h, bool verbose)
{
    tic_settings * settings = NULL;
    int rc=0;

    #define PRINT_SETTING(x) \
        do { \
            INFO("  %-32s = %d\n", #x, tic_settings_get_##x(settings)); \
        } while (0)

    tic_get_settings(motor[h].tic_handle, &settings);

    if (verbose) {
        INFO("SETTINGS ...\n");
        PRINT_SETTING(product);
        PRINT_SETTING(control_mode);
        PRINT_SETTING(never_sleep);
        PRINT_SETTING(disable_safe_start);
        PRINT_SETTING(ignore_err_line_high);
        PRINT_SETTING(auto_clear_driver_error);
        PRINT_SETTING(soft_error_response);
        PRINT_SETTING(soft_error_position);
        PRINT_SETTING(command_timeout);
        PRINT_SETTING(vin_calibration);
        PRINT_SETTING(current_limit);
        PRINT_SETTING(current_limit_code);
        PRINT_SETTING(current_limit_during_error);
        PRINT_SETTING(current_limit_code_during_error);
        PRINT_SETTING(step_mode);
        PRINT_SETTING(decay_mode);
        PRINT_SETTING(max_speed);
        PRINT_SETTING(starting_speed);
        PRINT_SETTING(max_accel);
        PRINT_SETTING(max_decel);
        PRINT_SETTING(invert_motor_direction);
    }

    // Calculation of max_speed and max_accel register values ...
    //
    // Telescope moves at 3 degrees per sec.
    // Stepper Shaft = 18 deg/sec
    //
    //                   6400 mcrosteps
    //     18 deg/sec x ----------------   => 320 microstep/sec
    //                      360 deg
    //
    //     max_speed register = 320 microsteps/sec x 10,000  => 3,200,000
    //
    // To accel to max speed in 10 secs ==> max_accel = max_speed / 1000
    //
    //      max_accel = 3,200,000 / (10 * 100)  =>  3,200
    //
    // To accel to max speed in 3.333 secs ==> max_accel = max_speed / 1000
    //
    //      max_accel = 3,200,000 / (3.333 * 100)  =>  9,600

    // check settings register values, and exit program if error; 
    // the settings should be programmed with ticgui

    #define CHECK_SETTING(x, expected_value) \
        do { \
            int actual_value; \
            actual_value = tic_settings_get_##x(settings); \
            if (actual_value != (expected_value)) { \
                ERROR("setting %s actual_value %d not equal expected %d\n", \
                      #x, actual_value, (expected_value)); \
                rc = -1; \
            } \
        } while (0)

    int expected_max_speed = rint(DEG2MICROSTEP(MAX_SPEED) * 10000);
    int expected_max_accel = rint(DEG2MICROSTEP(MAX_ACCEL) * 100);
    INFO("expected_max_speed/accel = %d %d\n", expected_max_speed, expected_max_accel);

    CHECK_SETTING(control_mode, 0);                      // 0 => Serial/I2C/USB
    CHECK_SETTING(disable_safe_start, 0);                // 0 => safe start not disabled
    CHECK_SETTING(soft_error_response, 2);               // 2 => decel to hold
    CHECK_SETTING(command_timeout, 1000);                // 1000 ms
    CHECK_SETTING(current_limit, 992);                   // 992 ma
    CHECK_SETTING(current_limit_code_during_error, 255); // 0xff => feature disabled
    CHECK_SETTING(step_mode, 5);                         // 5 => 1/32 step
    CHECK_SETTING(decay_mode, 2);                        // 0 => mixed, 1 => slow, 2 => fast   (T825)
    CHECK_SETTING(max_speed, expected_max_speed);        // 3200000 => shaft 18 deg/sec
    CHECK_SETTING(starting_speed, 0);                    // 0 => disallow instant accel or decel
    CHECK_SETTING(max_accel, expected_max_accel);        // 9600 => accel to max speed in 3.333 secs
    CHECK_SETTING(max_decel, 0);                         // 0 => max_decel is same as max_accel
    CHECK_SETTING(invert_motor_direction,0);             // 0 => do not invert direction   

    tic_settings_free(settings);
    return rc;
}

static void motor_inspect_variables(int h, bool verbose)
{
    tic_variables * variables = NULL;

    #define PRINT_VARIABLE(x) \
        do { \
            INFO("  %-32s = %d\n", #x, tic_variables_get_##x(variables)); \
        } while (0)

    tic_get_variables(motor[h].tic_handle, &variables, true);

    if (verbose) {
        INFO("VARIABLES ...\n");
        PRINT_VARIABLE(operation_state);
        PRINT_VARIABLE(energized);
        PRINT_VARIABLE(position_uncertain);
        PRINT_VARIABLE(error_status);
        PRINT_VARIABLE(errors_occurred);
        PRINT_VARIABLE(planning_mode);
        PRINT_VARIABLE(target_position);
        PRINT_VARIABLE(target_velocity);
        PRINT_VARIABLE(max_speed);
        PRINT_VARIABLE(starting_speed);
        PRINT_VARIABLE(max_accel);
        PRINT_VARIABLE(max_decel);
        PRINT_VARIABLE(current_position);
        PRINT_VARIABLE(current_velocity);
        PRINT_VARIABLE(acting_target_position);
        PRINT_VARIABLE(time_since_last_step);
        PRINT_VARIABLE(device_reset);
        PRINT_VARIABLE(vin_voltage);
        PRINT_VARIABLE(up_time);
        PRINT_VARIABLE(step_mode);
        PRINT_VARIABLE(current_limit);
        PRINT_VARIABLE(current_limit_code);
        PRINT_VARIABLE(decay_mode);
        PRINT_VARIABLE(input_state);
    }

    tic_variables_free(variables);
}

static char * motor_operation_state_str(int op_state)
{
    switch (op_state) {
    case 0: return "RESET";
    case 2: return "DE_ENERG";
    case 4: return "SOFT_ERR";
    case 6: return "WT_ERR_LN";    // waiting for error line
    case 8: return "START_UP";     // starting up
    case 10: return "NORMAL";
    }
    return "INVALID";
}

static char * motor_error_status_str(int err_stat) 
{
    static char str[100];
    char *p = str;

    if (err_stat == 0) return "NO_ERR";

    if (err_stat & (1<<0)) p += sprintf(p,"%s", "DEENERG ");      // "Intentionally_de-energized"
    if (err_stat & (1<<1)) p += sprintf(p,"%s", "DRVR_ERR ");     // "Motor_driver_error"
    if (err_stat & (1<<2)) p += sprintf(p,"%s", "LOW_VIN ");      // "Low_VIN"
    if (err_stat & (1<<3)) p += sprintf(p,"%s", "KILL_SW ");      // "Kill_switch_active"
    if (err_stat & (1<<4)) p += sprintf(p,"%s", "INP_INVLD ");    // "Required_input_invalid"
    if (err_stat & (1<<5)) p += sprintf(p,"%s", "SER_ERR ");      // "Serial_error"
    if (err_stat & (1<<6)) p += sprintf(p,"%s", "CMD_TOUT ");     // "Command_timeout"
    if (err_stat & (1<<7)) p += sprintf(p,"%s", "SS_VIOL ");      // "Safe_start_violation"
    if (err_stat & (1<<8)) p += sprintf(p,"%s", "ERR_LINE ");     // "ERR_line_high"

    if (p == str) return "INVLD";

    return str;
}    

#ifdef MOTOR_UNIT_TEST
#include <readline/readline.h>
#include <readline/history.h>

static void motor_unit_test(void)
{
    #define MAX_ARGV 10
    char * line = NULL;
    int    argc;
    char * argv[MAX_ARGV];
    char   line_copy[200];
    char   time_str[100];
    int    h;

    while (true) {
        // print prompt and read cmd/args
        free(line);
        if ((line = readline("DEBUG> ")) == NULL) {
            break;
        }
        if (line[0] != '\0') {
            add_history(line);
        }

        // parse line to argc/argv
        memset(argv, 0, sizeof(argv));
        strcpy(line_copy, line);
        argc = 0;
        while (true) {
            char * token = strtok(argc==0?line:NULL, " \n");
            if (token == NULL || argc == MAX_ARGV) {
                break;
            }
            argv[argc++] = token;
        }

        // if blank line then continue
        if (argc == 0) {
            continue;
        }

        // print to unit test log files
        for (h = 0; h < max_motor_devices; h++) {  
            fprintf(fp_unit_test[h], "%s %s\n", 
                    time2str(time_str, get_real_time_us(), false, true, true),
                    line_copy);
        }

        // process cmd
        if (strcmp(argv[0], "open") == 0) {
            int rc;
            rc = motor_open_all();
            INFO("rc %d\n", rc);
        } else if (strcmp(argv[0], "close") == 0) {
            motor_close_all();
        } else if (strcmp(argv[0], "stop") == 0) {
            motor_request_all_stop();
            motor_wait_for_all_stopped();
        } else if (strcmp(argv[0], "set") == 0) {
            int h, mstep;
            if (argv[1] == NULL || argv[2] == NULL) {
                ERROR("expected handle and mstep\n");
                continue;
            }
            if (sscanf(argv[1], "%d", &h) != 1) {
                ERROR("invalid handle '%s'\n", argv[1]);
                continue;
            }
            if (sscanf(argv[2], "%d", &mstep) != 1) {
                ERROR("invalid mstep '%s'\n", argv[2]);
                continue;
            }
            motor_set_pos(h, mstep);
        } else if (strcmp(argv[0], "adv") == 0) {
            #define KEY_REPEAT_INTVL (1./36.)  // sec
            int i, h, count=10/KEY_REPEAT_INTVL;
            int curr_pos1, curr_pos2;
            tic_variables * v;

            if (argv[1] == NULL) {
                ERROR("expected handle\n");
                continue;
            }
            if (sscanf(argv[1], "%d", &h) != 1) {
                ERROR("invalid handle '%s'\n", argv[1]);
                continue;
            }

            INFO("adv start: %d calls at %f sec interval\n", count, KEY_REPEAT_INTVL);
            for (i = 0; i < count; i++) {
                motor_adv_pos(h, 
                              DEG2MICROSTEP(MAX_SPEED * KEY_REPEAT_INTVL), // microsteps
                              DEG2MICROSTEP(5));                           // max microsteps ahead
                usleep(KEY_REPEAT_INTVL * 1000000);
            }
            INFO("adv done\n");
            fprintf(fp_unit_test[h], "%s adv done\n", 
                    time2str(time_str, get_real_time_us(), false, true, true));

            tic_get_variables(motor[h].tic_handle, &v, false);
            curr_pos1 = CURRENT_POSITION(v);
            tic_variables_free(v);

            motor_wait_for_stopped(h);

            tic_get_variables(motor[h].tic_handle, &v, false);
            curr_pos2 = CURRENT_POSITION(v);
            tic_variables_free(v);

            INFO("adv overshoot %d mstep (%f deg)\n",
                 curr_pos2 - curr_pos1,
                 MICROSTEP2DEG(curr_pos2 - curr_pos1));
            fprintf(fp_unit_test[h], "%s adv overshoot %d mstep (%f deg)\n", 
                    time2str(time_str, get_real_time_us(), false, true, true),
                    curr_pos2 - curr_pos1,
                    MICROSTEP2DEG(curr_pos2 - curr_pos1));
        } else if (strcmp(argv[0], "advms") == 0) {
            int h, mstep;

            if (argv[1] == NULL || argv[2] == NULL) {
                ERROR("expected handle and microsteps\n");
                continue;
            }
            if (sscanf(argv[1], "%d", &h) != 1) {
                ERROR("invalid handle '%s'\n", argv[1]);
                continue;
            }
            if (sscanf(argv[2], "%d", &mstep) != 1) {
                ERROR("invalid mstep '%s'\n", argv[2]);
                continue;
            }

            INFO("advancing %d by %d mstep (%f deg)\n",
                 h, mstep, MICROSTEP2DEG(mstep));

            motor_adv_pos(h, mstep, DEG2MICROSTEP(10));

        } else {
            ERROR("invalid command '%s'\n", argv[0]);
        }

        // print to unit test log file
        for (h = 0; h < max_motor_devices; h++) {
            fprintf(fp_unit_test[h], "%s done\n", 
                    time2str(time_str, get_real_time_us(), false, true, true));
        }
    }

    exit(0);
}
#endif

// -----------------  CAMERA ----------------------------------------------

bool cam_thread_running;
bool cam_thread_exit_req;
bool cam_reset_req;

static void cam_exit(void);
static void * cam_thread(void * cx);

static int cam_init(void) 
{
    pthread_t thread;

    pthread_create(&thread, NULL, cam_thread, NULL);
    atexit(cam_exit);
    return 0;
}

static void cam_exit(void)
{
    cam_thread_exit_req = true;
    while (cam_thread_running) {
        usleep(1000);
    }
}

static int cam_reset(void)
{
    cam_reset_req = true;
    return 0;
}

static void * cam_thread(void * cx)
{
    int rc;
    unsigned char * ptr;
    unsigned int len;
    size_t cmp_len;
    int act_fmt, act_width, act_height;
    double act_tpf;
    uint64_t time_now_us, time_last_cam_ctrlrs_get_all=0;

    static char msg_buffer[1000000];
    msg_t *msg = (msg_t*)msg_buffer;
    msg_cam_img_data_t *msg_data = (msg_cam_img_data_t*)msg->data;

    cam_thread_running = true;

re_init:
    // initialize camera
    while (true) {
        if (cam_thread_exit_req) {
            goto done;
        }

        rc = cam_initialize(FMT_MJPG, 1280, 960, 0.2, &act_fmt, &act_width, &act_height, &act_tpf);
        if (rc == 0) {
            cam_reset_req = false;
            break;
        }

        sleep(1);
    }

    while (true) {
        // check for time to exit
        if (cam_thread_exit_req) {
            goto done;
        }

        // check for request to reset the webcam
        if (cam_reset_req) {
            goto re_init;
        }

        // get cam buff:
        // if this fails then re-initialize camera
        rc = cam_get_buff(&ptr, &len);
        if (rc != 0) {
            goto re_init;
        }

        // if not connected then 
        //   zero cam_img_receipt_id and cam_img_lastsnd_id
        //   return the cam buff to the driver
        //   continue
        // endif
        if (!connected) {
            cam_img_receipt_id = 0;
            cam_img_lastsnd_id = 0;
            cam_put_buff(ptr);
            continue;
        }

        // if we haven't received acknowledgement that the last cam_img sent 
        // has been received then discard this cam_img; the reason is to not 
        // flood a slow network connection with more than it can take
        if (cam_img_receipt_id != cam_img_lastsnd_id &&
            cam_img_receipt_id != cam_img_lastsnd_id - 1)
        {
            WARN("discarding cam_img, receipt_id=%d lastsnd_id=%d\n", // XXX change to debug lvl
                 cam_img_receipt_id, cam_img_lastsnd_id);             //     or print every 100 discards
            cam_put_buff(ptr);
            continue;
        }
        cam_img_lastsnd_id++;

        // prepare the msg
        switch (act_fmt) {
        case FMT_MJPG:
            msg->id = MSGID_CAM_IMG;
            msg->data_len         = sizeof(msg_cam_img_data_t) + len;
            msg_data->pixel_fmt   = PIXEL_FMT_YUY2;
            msg_data->compression = COMPRESSION_JPEG_YUY2;
            msg_data->width       = act_width;
            msg_data->height      = act_height;
            msg_data->img_id      = cam_img_lastsnd_id;;
            memcpy(msg_data->data, ptr, len);
            break;
        case FMT_YU12:
#if 0
            // not compressed
            msg->id = MSGID_CAM_IMG;
            msg->data_len         = sizeof(msg_cam_img_data_t) + len;
            msg_data->pixel_fmt   = PIXEL_FMT_IYUV;
            msg_data->compression = COMPRESSION_NONE;
            msg_data->width       = act_width;
            msg_data->height      = act_height;
            msg_data->img_id      = cam_img_lastsnd_id;
            memcpy(msg_data->data, ptr, len);
#else
            // lzo compressed
            cmp_len = sizeof(msg_buffer) - sizeof(msg_t) - sizeof(msg_cam_img_data_t);
            compress(ptr, len, msg_data->data, &cmp_len);
            msg->id = MSGID_CAM_IMG;
            msg->data_len         = sizeof(msg_cam_img_data_t) + cmp_len;
            msg_data->pixel_fmt   = PIXEL_FMT_IYUV;
            msg_data->compression = COMPRESSION_LZO;
            msg_data->width       = act_width;
            msg_data->height      = act_height;
            msg_data->img_id      = cam_img_lastsnd_id;
#endif
            break;
        default:
            FATAL("unsupported pixel_fmt 0x%x\n", act_fmt);
        }

        // send the msg
        comm_send_msg(msg);

        // put buff
        cam_put_buff(ptr);

        // once per second get all of the ctrl values and send them to tcx
        time_now_us = microsec_timer();
        if (time_now_us - time_last_cam_ctrlrs_get_all > 900000) {
            time_last_cam_ctrlrs_get_all = time_now_us;
            msg->data_len = sizeof(msg_buffer) - sizeof(msg_t);
            if (cam_ctrls_get_all((cam_query_ctrls_t*)msg->data, &msg->data_len) == 0) {
                msg->id = MSGID_CAM_CTRLS_GET_ALL;
                comm_send_msg(msg);
            }
        }
    }

done:
    cam_thread_running = false;
    return NULL;
}
