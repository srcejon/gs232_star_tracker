# gs232_star_tracker
Program to track stars using a Yaesu GS232 rotator with Stellarium telescope control support

Build instructions
==================

`make all`

Command line options
====================

* `--ra decimal_hours`           - J2000 Right Ascension of star to track.
* `--dec decimal_hours`          - J2000 Declination of star to track.
* `--latitude decimal_degress`   - Latitude of rotator. Default is 51.507572 (for London).
* `--longitude decimal_degress`  - Longitude of rotator. Default is -0.127772.
* `--update seconds`             - Update interval of rotator in seconds. Default is 5.
* `--serial-port device`         - Serial-port device name to use for connection to rotator. Default is /dev/ttyS0.
* `--ip-port port_number`        - TCP/IP port number to listen for connections from Stellarium on. Default is 10001.
* `--verbose`                    - Print out received and calculated coordinates.

Typical Usage With Stellarium
=============================

Run the program specifying the latitude and longitude of the rotator. Assuming
you are in London:

`./gs232_star_tracker --latitude 51.507572 --longitude -0.127772 --serial-port /dev/ttyS2`

The program will then wait for the RA/Dec of star a to track from Stellarium.

Then in Stellarium:

- Enable Telescope Control plugin and restart
- Configure a telescope
- Set Telescope controlled by "External softare or a remote computer"
- Set Coordinate system to "J2000 (default)"
- Press Connect
- Enter RA/Declination or press "Current object" to get RA/Dec of currently selected object
- Press "Slew" to send the RA/Dec to gs232_star_tracker

Typical Usage Without Stellarium
================================

Run the program specifying the RA/Dec of the star to track. For example, to
track the Pulsar B0329+54, from a location in Sydney:

`./gs232_star_tracker --ra 3.549819 --dec 54.5791666 --latitude -33.856322 --longitude 151.215297`

The program should start tracking the star immediately. (You can still then
connect via Stellarium to update the target star.)

Obtaining Latitude and Longitude
================================

Latitude and longitude should be specified in decimal degress. Easy way to get
these is to:

- Look up your location in Google Maps.
- Left click to select the location.
- Right click and select "What's here?" from the menu.
- Latitude and longitude in decimal degrees should be displayed for that location.

