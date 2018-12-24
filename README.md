# Overview

The goal of this project is to retrofit a homemade 4 1/4 inch reflector
telescope, that was built about 1970. The mirror was ground from an
Edmund Scientific mirror grinding kit. The stand, equatorial mount, and
housing were custom made.

I plan to add motor drives to both axis, and a camera. This program will
drive the telescope, track an object, and display the image from the 
camera. This program will also include a sky chart.

# Sky Chart

The Sky Chart function is complete.

Build and run telescope controller program (tcx):

* Download and build this program. I currently use Linux Fedora-26.
* Review the sky_data/README file and download the stellar and solar system data
using the get_stellar_data and get_solar_sys_data shell scripts.
* Export environment variables MY_LAT and MY_LONG. These are your location in decimal degrees. East longitude is positive.
* I use display resolution 1920x1080; others should work.
* Run program: ./tcx

Using tcx. XXX TBD

* You should see three panes:

  * Az/El Pane across the top of the display
  * Control Pane at bottom left
  * Sky View Pane next to the Control Pane


* Az/El and Sky View Pane Controls:

  * Mouse Left Click or Arrow Keys to pan
  * Mouse Wheel or 'z' or 'Z' to zoom
  * 'M' or 'm' to adjust minimum magnitude of displayed objects
  * PgUp resets display to default 
  * PgDn resets display to default except Az/El pane shows objects above and below horizon
  * Mouse Right Click to select an object; Esc to cancel (only avail in Az/El Pane)
  * 'T' enables tracking the selected object; 't' cancel
  * '1' current; '2' pause; '3' fwd; '4' rew; '5' sid day fwd; '6' sid day rew


* Control Pane Commands (most useful is 'sel')

  * sel [<object_name>]: select an object by name, example "sel mars"
  * trk <on|off>: enable or disable tracking of selected object
  * reset [<all_sky>]: reset display to default
  * zoom <1..52>
  * mag <min_display_magnitude>
  * quit


* Custom Place Marks can be added, refer to sky_data/place_marks.dat.

Sky Chart testing:

* Refer to the unit_test_algorithms routine in sky.c.

Credits:

* Stellar Data: The Astronomy Nexus  http://www.astronexus.com/
* Solar Sys Data: telnet horizons.jpl.nasa.gov 6775
* RADEC to AZEL: references to the Nasa and Berkeley web sites are included in the code.
* Wikipedia

# Telescope Control

UNDER CONSTRUCTION
