### Overview - UNDER CONSTRUCTION

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
function will continue to operate even when the Stepper-Motor-Interface program 
is not being used.

Required environment variables:
* TCX_LAT=decimal-latitude
* TCX_LONG=decimal-longitude-west-negative
* TCX_AZ_CAL_POS=decimal-degrees-azimuth-of-calibration-spot
* TCX_EL_CAL_POS=decimal-degrees-elevation-of-calibration-spot
* TCX_CTLR_IP=name-or-ip-address-of-host-running-stepper-motor-intfc-pgm

When tcx starts it reads files containing the locations of objects. Refer to 
sky_data/NOTES for how to create these files. The files read are:
* sky_data/hygdata_v3.csv: star positions
* sky_data/solar_sys_data.csv: planet positions
* sky_data/place_marks.dat: other object positions

After reading the sky_data, tcx creates a window containing 3 panes:
* sky-az-el: top half of window - displays the sky objects in azimuth/elevation coordinates
* sky-view: lower left - a less distorted sky view with same center as sky-az-el pane
* tele-ctl: lower middle - used to control the telescope and intended to eventually 
  display the telescope camera image

Each pane has it's own user interface. Left click to select the pane. The pane's border will
become green to indicate it is the selected pane. The following sections describe the controls for
each pane.

Sky-az-el pane controls:
* cmdline:
  * ident <objname>|off - locate the named object and display in orange
  * trk <objname>|ident|curr|off - the named object is held in the center of the pane
  * tstep delta_t|sunrise[+/-h.hh]|sunset[+/-h.hh]|sidday|h.hhh: timestep for the ALT-3,4,5,6 single key cmds described below
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

### Stepper-Motor-Interface Program (ctlr)

I run this program on a battery powered Raspberry Pi which will be attached to the base
of the telescope. The Raspberry Pi (running the ctlr program) communicates to the laptop 
(running the tcx program) via WIFI. The ctlr program receives commands from the tcx program
to set the telescope's azimuth and elevation.

### Building and Running the Telescope-Control Program (tcx)

This program builds warning free on Fedora-26. To build simply run make.

Follow the procedure described in sky_data/NOTES to download the stellar and solar-sys-data files.

Create the following environment variables, described above;
TCX_LAT, TCX_LONG, TCX_AZ_CAL_POS, TCX_EL_CAL_POS, TCX_CTLR_IP.

I use display resolution 1920x1080; other display resolutions will probably work too.

./tcx runs the program.

### Building and Running the Stepper-Motor-Interface Program (ctlr)

Perform the following steps in the Raspberry Pi, or wherever you plan to run 
the ctlr program.

Follow the directions in devel/pololu/NOTES for
'BUILDING POLOLU TIC SOFTWARE FROM SOURCE ON FEDORA 26 & RASBIAN'.

make -f Makefile.ctlr

If desired, follow the directions in comments near the beginning of ctlr.c
'to automatically start the ctlr program when the Raspberry Pi boots'.

### Credits

* Stellar Data: The Astronomy Nexus  http://www.astronexus.com/
* Solar Sys Data: telnet horizons.jpl.nasa.gov 6775
* RADEC to AZEL: references to the Nasa and Berkeley web sites are included in the code.
* Wikipedia

