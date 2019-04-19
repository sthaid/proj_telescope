### Overview

The goal of this project is to retrofit a homemade 4 1/4 inch reflector
telescope, that was built about 1970. The mirror was ground from an
Edmund Scientific mirror grinding kit. The stand, equatorial mount, and
housing were custom made.

I plan to add motor drives to both axis (using azimuth/elevation orientation), 
and a camera. This program will drive the telescope, track an object, and display 
the image from the camera. This program will also include a sky chart.

### Telescope-Control Program (tcx)

Runs on linux desktop or laptop. I currently use Fedora 26.

Connects to the Stepper-Motor-Interface program (ctlr) via IPv4 TCP sockets. 
Sends messages to the Stepper-Motor-Interface program to drive telescope 
stepper motors to the desired azimuth and elevation. This program's sky chart 
function continues to operate when the Stepper-Motor-Interface program 
is not being used.

Required environment variables:
* TCX_LAT=decimal-latitude
* TCX_LONG=decimal-longitude-west-negative
* TCX_CTLR_IP=name-or-ip-address-of-host-running-stepper-motor-intfc-pgm
* TCX_AZ_TELE_LEG_1=azimuth of a specific telescope leg, used by the software
to limit telescope positioning based on telescope's mechanical attributes

When tcx starts it reads files containing the locations of objects. Refer to 
sky_data/NOTES for how to create these files. The files read are:
* sky_data/hygdata_v3.csv: star positions
* sky_data/solar_sys_data.csv: planet positions
* sky_data/place_marks.dat: other object positions
* sky_data/cal_locs.dat: calibration position(s)

After reading the sky_data, tcx creates a window containing several panes:
* sky-az-el: top half of window - displays the sky objects in azimuth/elevation coordinates
* tele-ctl: lower left - used to control the telescope and display telescope camera image
* sky-view: lower middle - a less distorted sky view with same center as sky-az-el pane
* camera-info: lower middle (overlays sky-view): webcam settings
* motor-info: azimuth / elevation stepper motor values (primarily for debugging problems)

Each pane has it's own user interface. Left click to select the pane. The pane's border will
become green to indicate it is the selected pane. The following sections describe the controls for
each pane. Use right click to select panes that overlay each other; right-click will demote
the current pane to the background.

Sky-az-el pane controls:
* cmdline:
  * trk \<objname\>|ident|curr|off - the named object is held in the center of the pane
  * ident \<objname\>|off - locate the named object and display in orange
  * tstep delta_t|sunrise[+/-h.hh]|sunset[+/-h.hh]|sidday|h.hhh: timestep for the ALT-3,4,5,6 single key cmds described below
  * azel \<az\> \<el\> - the center of the pane is set to the specified az / el
  * quit - terminate
* mouse left click or arrow keys - pan
* mouse wheel - zoom
* mouse right click - identify an object
* ALT-m or ALT-M - select magnitude
* PgUp - reset using 90 degree elevation scale
* PgDn - reset using 180 degree elevation scale
* Home - go to calibration az/el specified in environment variables
* ALT-1,2,3,4,5,6 - used in conjunction with 'tstep' cmdline command: (1) curr-time, (2) pause,
  (3) reverse by tstep, (4) forward by tstep, (5) single step reverse, (6) single step forward
* ALT-q - terminate

Sky-view pane controls:
* mouse left click or arrow keys - pan
* mouse wheel - zoom

Tele-ctl pane controls:
* motors open/close - energizes or de-energizes the motors
* calibrate/uncalibrate - associate the current telescope position with the center of the sky-az-el pane
* track enable/disable - enable the telescope to track the center of the sky-az-el pane
* arrow keys - when uncalibrated these keys position the telescope; when calibrated these keys
  provide fine adjustment of the telescope position of up to 1 degree
* search - when tracking is acquired, search can be used to position the telescope
  in a spiral pattern to help locate the desired object
* shutdown ctlr - issues shutdown command on the ctlr

### Stepper-Motor-Interface Program (ctlr)

I run this program on a Raspberry Pi which is attached to the base of the telescope. 
The Raspberry Pi (running the ctlr program) communicates to the laptop 
(running the tcx program) via WIFI. The ctlr program receives commands from the tcx program
to set the telescope's azimuth and elevation.

### Building and Running the Telescope-Control Program (tcx)

This program builds warning free on Fedora-26. To build simply run make.

Follow the procedure described in sky_data/NOTES to decompress the hygdata_v3.csv.gz
and solar_sys_data.csv.gz. These files contain the stellr and solar system object
coordinates respectively.

Create the following environment variables, described above;
TCX_LAT, TCX_LONG, TCX_CTLR_IP, and TCX_AZ_TELE_LEG_1.

I prefer to use display resolution 1920x1080; other display resolutions will work too.

./tcx runs the program.

### Building and Running the Stepper-Motor-Interface Program (ctlr)

Perform the following steps in the Raspberry Pi, or wherever you plan to run 
the ctlr program.

Follow the directions in devel/pololu/NOTES for
'BUILDING POLOLU TIC SOFTWARE FROM SOURCE ON FEDORA 26 & RASBIAN'.

make -f Makefile.ctlr

If desired, follow the directions in comments near the beginning of ctlr.c
'to automatically start the ctlr program when the Raspberry Pi boots'.

### View-Planner Program (vp)

This program helps to identify viewing times for specified objects. Inputs are:
* an area of the sky: az range, el range
* time span: for example 'sunset+1 sunset+6'
* number of days
* list of objects: for example 'mercury,venus,mars'

Example:

    $ ./vp 0 360 15 90 sunset sunset+4 7 mercury
    az_range   = 0 360
    el_range   = 15 90
    start_time = sunset
    end_time   = sunset+4
    max_day    = 7
    lat/long   = 42.422986 -71.623798
    max_obj    = 1 : Mercury 
    start_date = 2/17/2019  (m/d/y)

                       Mercury 
    02/22/19 17:26:48   251 15 

                       Mercury 
    02/23/19 17:28:04   252 16 


### Credits

* Stellar Data: The Astronomy Nexus  http://www.astronexus.com/
* Solar Sys Data: telnet horizons.jpl.nasa.gov 6775
* RADEC to AZEL: references to the Nasa and Berkeley web sites are included in the code.
* Wikipedia

