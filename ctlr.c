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

// XXX review FATALs
// XXX motor err chks

#include "common.h"
#include <tic.h>

//
// defines
//

#define UNIT_TEST

//
// typedefs
//

//
// variables
//

//
// prototypes
//

// general routines
void sig_handler(int sig);

// message routines
int comm_init(void);
void comm_send_msg(msg_t * msg);

// motor routines
int motor_init(void);
void motor_exit(void);
int motor_open_all(void);
void motor_close_all(void);
int motor_open(int h);
void motor_close(int h);
int motor_set_pos(int h, double deg);
int motor_adv_pos(int h, double deg, double max_deg);
int motor_request_stop(int h);
int motor_wait_for_stopped(int h);
int motor_stop_all(void);
void motor_unit_test(void);

// -----------------  MAIN  -----------------------------------------------

bool signal_rcvd;

int main(int argc, char **argv)
{
    struct sigaction act;
    int rc;

    // register signal handler
    memset(&act, 0,sizeof(act));
    act.sa_handler = sig_handler;
    sigaction(SIGINT, &act, NULL);
    sigaction(SIGTERM, &act, NULL);

    // register motor exit handler
    atexit(motor_exit);

    // motor initialize
    rc = motor_init();
    if (rc < 0) {
        FATAL("motor_init failed\n");
    }

    // comm initialize
    rc = comm_init();
    if (rc < 0) {
        FATAL("comm_init failed\n");
    }

#ifdef UNIT_TEST
    // unit_test
    motor_unit_test();
#endif

    // pause until signal received
    while (!signal_rcvd) pause();
    INFO("terminating\n");
    return 0;
}

void sig_handler(int sig)
{
    signal_rcvd = sig;
}

// -----------------  COMM  -----------------------------------------------

//
// variables
//

int sfd = -1;
bool connected;

//
// prototypes
//

void * comm_ctlr_thread(void * cx);
void * comm_heartbeat_thread(void * cx);
void comm_process_recvd_msg(msg_t * msg);

//
// code
//

int comm_init(void)
{
    pthread_t thread_id;

    pthread_create(&thread_id, NULL, comm_ctlr_thread, NULL);
    pthread_create(&thread_id, NULL, comm_heartbeat_thread, NULL);

    return 0;
}

void * comm_ctlr_thread(void * cx)
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
    comm_send_msg(&msg);

    // on new connection should first recv MSG_ID_CONNECTED
    len = do_recv(sfd, &msg, sizeof(msg_t));
    if (len != sizeof(msg_t)) {
        ERROR("recvd initial msg with invalid len %d, %s\n", len, strerror(errno));
        goto lost_connection;
    }
    if (msg.id != MSG_ID_CONNECTED) {
        ERROR("recvd invalid initial msg, id=%lld\n", msg.id);
        goto lost_connection;
    }
    connected = true;

    // receive msgs from tele_ctlr, and
    // process them
    while (true) {
        // recv msg  
        len = do_recv(sfd, &msg, sizeof(msg_t));
        if (len != sizeof(msg_t)) {
            ERROR("recvd msg with invalid len %d, %s\n", len, strerror(errno));
            break;
        }

        // process the recvd msg
        comm_process_recvd_msg(&msg);
    }

lost_connection:
    // lost connection; reconnect
    connected = false;
    sfd_temp = sfd;
    sfd = -1;
    close(sfd_temp);
    ERROR("lost connection from tcx, attempting to reconnect\n");
    goto reconnect;
}

void comm_process_recvd_msg(msg_t * msg)
{
    INFO("received %s\n", MSG_ID_STR(msg->id));
#if 0
    // XXX
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

void comm_send_msg(msg_t * msg)
{
    int len;

    INFO("sending %s\n", MSG_ID_STR(msg->id));

    len = do_send(sfd, msg, sizeof(msg_t));
    if (len != sizeof(msg_t)) {
        ERROR("send failed, len=%d, %s\n", len, strerror(errno));
    }
}

void * comm_heartbeat_thread(void * cx)
{
    msg_t msg;

    memset(&msg,0,sizeof(msg_t));
    msg.id = MSG_ID_HEARTBEAT;

    while (true) {
        if (connected) {
            comm_send_msg(&msg);
        }
        usleep(200000);
    }
}

// -----------------  MOTOR  ----------------------------------------------

//
// defines
//

#define MAX_MOTOR 4

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

#define MAX_SPEED 18.0  // deg/sec
#define MAX_ACCEL 5.4   // deg/sec/sec

#define DEG2MICROSTEP(d)      (rint((d) * ((200.*32.) / 360.)))
#define MICROSTEP2DEG(mstep)  ((mstep) * (360. / (200.*32.)))

#define MICROSTEPVEL2DEGPERSEC(mstepv)  ((mstepv) * (360. / (200.*32.)) / 10000.)

//
// variables
//

struct motor_s {
    char serial_number[50];
    tic_handle *tic_handle;
} motor[MAX_MOTOR];

pthread_mutex_t motor_mutex = PTHREAD_MUTEX_INITIALIZER;

tic_device ** motor_devices;
size_t max_motor_devices;

bool motor_initialized;
bool motor_keepalive_thread_running;
bool motor_getstatus_thread_running;

#ifdef UNIT_TEST
FILE *fp_unit_test[MAX_MOTOR];
#endif

//
// prototypes
//

void * motor_get_status_thread(void * cx);
void * motor_getstatus_thread(void * cx);
void * motor_keepalive_thread(void * cx);
void motor_check_settings(int h, bool verbose);
void motor_check_variables(int h, bool verbose);
char * motor_operation_state_str(int op_state);
char * motor_error_status_str(int err_stat);

// ---- init & exit ----

// intended to be called once at startup
int motor_init(void)
{
    tic_error * err;
    pthread_t thread_id;
    int h;

    // fatal error if already initialized
    if (motor_initialized) {
        FATAL("already initialized\n");
    }

    // set initialized flag
    motor_initialized = true;

    // get list of connected devices;
    err = tic_list_connected_devices(&motor_devices, &max_motor_devices);
    if (err) {
        FATAL("tic_list_connected_devices, %s\n", tic_error_get_message(err));
    }

    // print list of connected devices
    INFO("number of devices found = %zd\n", max_motor_devices);
    for (h = 0; h < max_motor_devices; h++) {
        INFO("device %d serial_number = %s\n", 
             h, tic_device_get_serial_number(motor_devices[h]));
    }

    // sanity check that there are not more motors connected than this program is
    // configured to support
    if (max_motor_devices > MAX_MOTOR) {
        FATAL("too many motors, %zd\n", max_motor_devices);
    }

#ifdef UNIT_TEST
    // open unit test log files
    for (h = 0; h < max_motor_devices; h++) {
        char fn[100];
        sprintf(fn, "ctrl_unit_test_%d.log", h);
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
void motor_exit(void)
{
    int i;

    // it not initialized just return
    if (!motor_initialized) {
        return;
    }

    // stop all motors
    motor_stop_all();

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

int motor_open_all(void)
{
    int h;

    INFO("called\n");

    for (h = 0; h < max_motor_devices; h++) {
        if (motor_open(h) < 0) {
            return -1;
        }
    }

    return 0;
}

void motor_close_all(void)
{
    int h;

    INFO("called\n");

    for (h = 0; h < max_motor_devices; h++) {
        if (motor[h].tic_handle == NULL) {
            continue;
        }
        motor_close(h);
    }
}

// ---- open and close single motor ----

int motor_open(int h)
{
    tic_error * err;
    tic_handle * tic_handle;

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
    // (use tic_gui to make corrections)
    motor_check_settings(h, false);
    motor_check_variables(h, false);

    // energize and exit_safe_start 
    tic_energize(tic_handle);
    tic_exit_safe_start(tic_handle);

    // release mutex
    pthread_mutex_unlock(&motor_mutex);

    // return success
    return 0;
}

void motor_close(int h)
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

int motor_set_pos(int h, double deg)
{
    tic_error * err;

    // check that motor[h] has been opened
    if (h >= MAX_MOTOR || motor[h].tic_handle == NULL) {
        ERROR("invalid handle %d\n", h);
        return -1;
    }

    // set motor position
    err = tic_set_target_position(motor[h].tic_handle, DEG2MICROSTEP(deg));
    if (err) {
        ERROR("tic_set_target_position %d, %s\n", h, tic_error_get_message(err));
        return -1;
    }

    // return success
    return 0;
}

int motor_adv_pos(int h, double deg, double max_deg)
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

    // ignore request if tgt_pos is already more than max_deg from curr_pos
    if (deg > 0 && tgt_pos - curr_pos > DEG2MICROSTEP(max_deg)) {
        return 0;
    }
    if (deg < 0 && tgt_pos - curr_pos < -DEG2MICROSTEP(max_deg)) {
        return 0;
    }
    if (deg == 0) {
        return 0;
    }

    // set new target position by adding to the current tgt_pos
    err = tic_set_target_position(motor[h].tic_handle, tgt_pos+DEG2MICROSTEP(deg));
    if (err) {
        ERROR("tic_set_target_position %d, %s\n", h, tic_error_get_message(err));
        return -1;
    }

    // success
    return 0;
}

// ---- stop motors ----

int motor_request_stop(int h)
{
    tic_variables * v;
    int curr_pos, curr_vel;
    double vel_degpersec, pos_deg, stop_pos_deg;

    // check that motor[h] has been opened
    if (h >= MAX_MOTOR || motor[h].tic_handle == NULL) {
        ERROR("invalid handle %d\n", h);
        return -1;
    }

    // get curr_pos and curr_vel
    tic_get_variables(motor[h].tic_handle, &v, false);
    curr_pos = CURRENT_POSITION(v);
    curr_vel = CURRENT_VELOCITY(v);
    tic_variables_free(v);

    // convert velocity and position to deg/sec and deg
    vel_degpersec = MICROSTEPVEL2DEGPERSEC(curr_vel);
    pos_deg = MICROSTEP2DEG(curr_pos);

    // determine pos to stop at based on max decel
    if (vel_degpersec >= 0) {
        stop_pos_deg = pos_deg + vel_degpersec * vel_degpersec / (2. * MAX_ACCEL);
    } else {
        stop_pos_deg = pos_deg - vel_degpersec * vel_degpersec / (2. * MAX_ACCEL);
    }

    // set pos
    motor_set_pos(h, stop_pos_deg);

    // success
    return 0;
}

int motor_wait_for_stopped(int h)
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
            INFO("motor %d is stopped\n", h);
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

int motor_stop_all(void)
{
    int h;

    INFO("request all motors stop\n");
    for (h = 0; h < max_motor_devices; h++) {
        if (motor[h].tic_handle == NULL) {
            continue;
        }
        motor_request_stop(h);
    }

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
    
// ---- motor_get_status_thread ----

void * motor_getstatus_thread(void * cx)
{
    int h;
    tic_variables * variables[MAX_MOTOR];

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

            tic_get_variables(motor[h].tic_handle, &variables[h], true);

            tic_variables *v = variables[h];
            if (ENERGIZED(v) == 0 && VOLTAGE_OK(VIN_VOLTAGE(v))) {
                INFO("energizing motor %d\n", h);
                tic_energize(motor[h].tic_handle);
                tic_exit_safe_start(motor[h].tic_handle);
            }

            // print when an error occurred
            //if (ERRORS_OCCURRED(v)) {
            //    ERROR("errors_occurred: %s\n", motor_error_status_str(ERRORS_OCCURRED(v)));
            //}
        }

#ifdef UNIT_TEST
        {
        for (h = 0; h < MAX_MOTOR; h++) {
            tic_variables *v = variables[h];
            char time_str[100];

            if (motor[h].tic_handle == NULL) {
                continue;
            }

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
        } }
#endif

        // XXX send status msg
        //         also some other status, such as
        //         all motors without error
        //         all at target (or near)
        //         for each motor, a flag if at / near target

        // free variables
        for (h = 0; h < MAX_MOTOR; h++) {
            if (variables[h] != NULL) {
                tic_variables_free(variables[h]);
                variables[h] = NULL;
            }
        }

        // release mutex
        pthread_mutex_unlock(&motor_mutex);

        // delay 1 sec
        usleep(1000000);
    }

    // terminate
    motor_getstatus_thread_running = false;
    return NULL;
}

// ---- motor_keepalive_thread ----

void * motor_keepalive_thread(void * cx)
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

void motor_check_settings(int h, bool verbose)
{
    tic_settings * settings = NULL;

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
    //                      360
    //
    //     max_speed register = 320 microsteps/sec x 10,000  => 3,200,000
    //
    // To accel to max speed in 10 secs ==> max_accel = max_speed / 1000
    //
    //      max_accel = 3,200,000 / 1000  =>  3,200

    // check settings register values, and exit program if error; 
    // the settings should be programmed with ticgui

    #define CHECK_SETTING(x, expected_value) \
        do { \
            int actual_value; \
            actual_value = tic_settings_get_##x(settings); \
            if (actual_value != (expected_value)) { \
                FATAL("ERROR setting %s actual_value %d not equal expected %d\n", \
                      #x, actual_value, (expected_value)); \
            } \
        } while (0)

    CHECK_SETTING(control_mode, 0);                      // 0 => Serial/I2C/USB
    CHECK_SETTING(disable_safe_start, 0);                // 0 => safe start not disabled
    CHECK_SETTING(soft_error_response, 2);               // 2 => decel to hold
    CHECK_SETTING(command_timeout, 1000);                // 1000 ms
    CHECK_SETTING(current_limit, 992);                   // 992 ma
    CHECK_SETTING(current_limit_code_during_error, 255); // 0xff => feature disabled
    CHECK_SETTING(step_mode, 5);                         // 5 => 1/32 step
    CHECK_SETTING(decay_mode, 2);                        // 2 => fast
    CHECK_SETTING(max_speed, 3200000);                   // 3200000 => shaft 18 deg/sec
    CHECK_SETTING(starting_speed, 0);                    // 0 => disallow instant accel or decel
    CHECK_SETTING(max_accel, 9600);                      // 9600 => accel to max speed in 3.333 secs
    CHECK_SETTING(max_decel, 0);                         // 0 => max_decel is same as max_accel
    CHECK_SETTING(invert_motor_direction,0);             // 0 => do not invert direction   

    tic_settings_free(settings);
}

void motor_check_variables(int h, bool verbose)
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

    // check voltage 
    int vin_voltage = tic_variables_get_vin_voltage(variables);
    if (!VOLTAGE_OK(vin_voltage)) {
        WARN("vin_voltage=%d not okay\n", vin_voltage);
    }

    tic_variables_free(variables);
}

char * motor_operation_state_str(int op_state)
{
    switch (op_state) {
    case 0: return "RESET";
    case 2: return "DEENERGIZED";
    case 4: return "SOFT_ERR";
    case 6: return "WAITING_FOR_ERR_LINE";
    case 8: return "STARTING_UP";
    case 10: return "NORMAL";
    }
    return "INVALID";
}

char * motor_error_status_str(int err_stat) 
{
    static char str[100];
    char *p = str;

    if (err_stat == 0) return "No_Err";

    if (err_stat & (1<<0)) p += sprintf(p,"%s","Intentionally_de-energized,");
    if (err_stat & (1<<1)) p += sprintf(p,"%s","Motor_driver_error,");
    if (err_stat & (1<<2)) p += sprintf(p,"%s","Low_VIN,");
    if (err_stat & (1<<3)) p += sprintf(p,"%s","Kill_switch_active,");
    if (err_stat & (1<<4)) p += sprintf(p,"%s","Required_input_invalid,");
    if (err_stat & (1<<5)) p += sprintf(p,"%s","Serial_error,");
    if (err_stat & (1<<6)) p += sprintf(p,"%s","Command_timeout,");
    if (err_stat & (1<<7)) p += sprintf(p,"%s","Safe_start_violation,");
    if (err_stat & (1<<8)) p += sprintf(p,"%s","ERR_line_high,");

    if (p == str) return "Invalid_Err_Stat";

    return str;
}    

#ifdef UNIT_TEST
#include <readline/readline.h>
#include <readline/history.h>

void motor_unit_test(void)
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
            motor_stop_all();
        } else if (strcmp(argv[0], "set") == 0) {
            int h;
            double deg;
            if (argv[1] == NULL || argv[2] == NULL) {
                ERROR("expected handle and shaft-degrees\n");
                continue;
            }
            if (sscanf(argv[1], "%d", &h) != 1) {
                ERROR("invalid handle '%s'\n", argv[1]);
                continue;
            }
            if (sscanf(argv[2], "%lf", &deg) != 1) {
                ERROR("invalid deg '%s'\n", argv[2]);
                continue;
            }
            motor_set_pos(h, deg);
        } else if (strcmp(argv[0], "adv") == 0) {
            #define KEY_REPEAT_INTVL (1./36.)  // sec xxx
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
                              MAX_SPEED * KEY_REPEAT_INTVL,       // degrees
                              5);                               // max degrees ahead
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

            INFO("adv overshoot %f deg\n",
                 MICROSTEP2DEG(curr_pos2 - curr_pos1));
            fprintf(fp_unit_test[h], "%s adv overshoot %f deg\n", 
                    time2str(time_str, get_real_time_us(), false, true, true),
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

            motor_adv_pos(h, 
                          MICROSTEP2DEG(mstep),  // degrees
                          10.);                  // max degrees ahead

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
