// This program demonstrates stepper motor position control 
// using the Pololu Tic T825 USB Multi-Interface Stepper Motor Controller.
//
// https://www.pololu.com/product/3130/resources

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <tic.h>

// defines

#define ERR_CHK(statement) \
    do { \
        tic_error * error; \
        error = statement; \
        if (error) { \
            fprintf(stderr, "ERROR: '%s' -> %s\n", #statement, tic_error_get_message(error)); \
            exit(1); \
        } \
    } while (0)

#define MIN_VIN_VOLTAGE 10000  // mv
#define MAX_VIN_VOLTAGE 15000

// variables

tic_handle * handle;
tic_settings * settings;
tic_variables * variables;
int signal_rcvd;

// prototypes

void open_tic(void);
void exit_handler(void);
void sig_handler(int sig);

void * keep_alive_thread(void * cx);
void * monitor_pos_thread(void * cx);
char * operation_state_str(int op_state);
char * error_status_str(int err_stat) ;

void check_settings(void);
void check_variables(void);

// -----------------  MAIN  ----------------------------------------------

int main(int argc, char **arg)
{
    int pos;
    pthread_t thread_id;
    struct sigaction act;

    // get handle to the tic device
    open_tic();

    // register exit handler, and signal handler
    atexit(exit_handler);
    
    memset(&act, 0,sizeof(act)); 
    act.sa_handler = sig_handler;
    sigaction(SIGINT, &act, NULL);
    sigaction(SIGTERM, &act, NULL);

    // reset the tic
    ERR_CHK(tic_reset(handle));
    ERR_CHK(tic_set_target_position(handle, 0));

    // print some settings and variables, and validate
    check_settings();
    check_variables();

    // create threads to:
    // - reset command timeout
    // - monitor position and status
    pthread_create(&thread_id, NULL, keep_alive_thread, NULL);
    pthread_create(&thread_id, NULL, monitor_pos_thread, NULL);

    // energize and exit_safe_start 
    ERR_CHK(tic_energize(handle));
    ERR_CHK(tic_exit_safe_start(handle));

    // slowly increment position
    for (pos = 0; pos < 200*32; pos++) {
        ERR_CHK(tic_set_target_position(handle, pos));
        usleep(2250000);  // 2.25 secs

        if (signal_rcvd) {
            fprintf(stderr, "terminating due to signal %d\n", signal_rcvd);
            return 1;
        }
    }

    // invoke exit_handler() when program terminates
    return 0;
}

void open_tic()
{
    tic_device ** list;
    size_t count;
    int i;

    // get list of connected devices;
    // this program supports just one device, so error if not 1 device
    ERR_CHK(tic_list_connected_devices(&list, &count));
    for (i = 0; i < count; i++) {
        fprintf(stderr, "device %d serial_number = %s\n", i, tic_device_get_serial_number(list[i]));
    }
    if (count != 1) {
        fprintf(stderr, "ERROR must be just one device, but count=%zd\n", count);
        exit(1);
    }

    // open handle to the tic device in list[0]
    ERR_CHK(tic_handle_open(list[0], &handle));

    // cleanup
    for (i = 0; i < count; i++) {
        tic_device_free(list[i]);
    }
    tic_list_free(list);
}

void exit_handler(void)
{
    fprintf(stderr, "exit_handler\n");

    // enter_safe_start and deenergize
    ERR_CHK(tic_enter_safe_start(handle));
    ERR_CHK(tic_deenergize(handle));

    // cleanup
    tic_settings_free(settings);
    tic_variables_free(variables);
    tic_handle_close(handle);
}

void sig_handler(int sig)
{
    signal_rcvd = sig;
}

// -----------------  THREADS  -------------------------------------------

void * keep_alive_thread(void * cx)
{
    while (true) {
        ERR_CHK(tic_reset_command_timeout(handle));
        usleep(100000);
    }
    return NULL;
}

void * monitor_pos_thread(void * cx)
{
    sleep(3);

    while (true) {
        ERR_CHK(tic_get_variables(handle, &variables, false));

        int operation_state    = tic_variables_get_operation_state(variables);
        int energized          = tic_variables_get_energized(variables);
        int position_uncertain = tic_variables_get_position_uncertain(variables);
        int error_status       = tic_variables_get_error_status(variables);
        int vin_voltage        = tic_variables_get_vin_voltage(variables);
        int target_position    = tic_variables_get_target_position(variables);
        int current_position   = tic_variables_get_current_position(variables);

        fprintf(stderr, "op_state=%d(%s) energized=%d pos_unc=%d err_status=0x%x(%s) mv=%d tgt_pos=%d curr_pos=%d\n",
                operation_state, operation_state_str(operation_state),
                energized,
                position_uncertain,
                error_status, error_status_str(error_status),
                vin_voltage,
                target_position,
                current_position);

        sleep(1);
    }
    return NULL;
}

char * operation_state_str(int op_state)
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

char * error_status_str(int err_stat) 
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

// -----------------  SETTINGS  ------------------------------------------

void check_settings(void)
{
    #define PRINT_SETTING(x) \
        do { \
            fprintf(stderr, "  %-32s = %d\n", #x, tic_settings_get_##x(settings)); \
        } while (0)

    tic_settings_free(settings);

    ERR_CHK(tic_get_settings(handle, &settings));

    fprintf(stderr, "SETTINGS ...\n");
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
                fprintf(stderr, "ERROR setting %s actual_value %d not equal expected %d\n", \
                        #x, actual_value, (expected_value)); \
                exit(1); \
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
    CHECK_SETTING(max_accel, 3200);                      // 3200 => accel to max speed in 10 secs
    CHECK_SETTING(max_decel, 0);                         // 0 => max_decel is same as max_accel
    CHECK_SETTING(invert_motor_direction,0);             // 0 => do not invert direction   
}

// -----------------  VARIABLES  -----------------------------------------

void check_variables(void)
{
    #define PRINT_VARIABLE(x) \
        do { \
            fprintf(stderr, "  %-32s = %d\n", #x, tic_variables_get_##x(variables)); \
        } while (0)

    tic_variables_free(variables);

    ERR_CHK(tic_get_variables(handle, &variables, false));

    fprintf(stderr, "VARIABLES ...\n");
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

    // check voltage
    int vin_voltage = tic_variables_get_vin_voltage(variables);
    if (vin_voltage < 10000 || vin_voltage > 15000) {
        fprintf(stderr, "ERROR variable vin_voltage=%d out of range %d to %d\n",
                vin_voltage, MIN_VIN_VOLTAGE, MAX_VIN_VOLTAGE);
        exit(1);
    }
}
