# GPS_locator_OLED_GNGGA
GPS NMEA offline drone locator using Arduino, OLED display, internal GPS and remote tracked GPS through radio

Used for tracking and locating lost drones.
There is an Android version of the tracker with cheap hardware solution, but this is a standalone solution with its own microcontroller, display, GPS etc.

A long range 433Mhz Radio connection used on tracked device providing GPS NMEA sentences can be both used locally by Flight Controller for navigation and OSD and for tracking by this tool.

Tracking process:
This tool has its own GPS receiver to know its own position too during the finding process and displays heading for compass navigation and distance for last known drone position.
Current setup can receive drone remote GPS data for about 1-2km range in air so drone can be tracked before a disarm failsafe event in air.
When drone crashed to the ground radio connection is lost, the device helps user to navigate to the last known coordinates near crash location and about 200-300meters range at ground radio connection could be established again to have a precise fresh location data if remote device still working and sending position data.

## Current Functionalities

* parsing NMEA GxGGA and GxRMC messages for 2 GPS units: remote tracked sentences coming from long range radio UART and a local GPS unit
* checking CRC
* distance sanitation
* collecting last known position, altitude etc data
* dislaying on OLED display the most usefull information for tracking like longitude latitude, bearing, distance, altitude
* displaying other usefull statistic data like number of satellites used, max speed in kph, maxdistance, UTC time etc.
* reseting serial connection and reconnecting recovery mode preiodically if no incoming data
* displaying debug and statistic info for number of radio/local GPS reconnects, buffer sizes, CRC fails, timestamp, battery voltage
* displaying NMEA packages
* buzzer sound beeps for radio serial communication so connection issues/out of range can be heard even wearing FPV glasses by sound
* push button to change screen
* BN-220 GNSS unit used configured for SBAS, GPS, Glonass, Galileo satellites too
* currently not using navigation calculation from the local GPS and microcontroller/sw calculates geographic data

## TODO

* HW: lost of LED outputs used for connection statuses etc. with not connected hardware devices
* SW: get rid of remaining String objects
* HW: removable antenna connector (or exchangable radio unit connector)
* SW: connect local GPS RX pin too and use that module for navigation calculation and parsing GNxRMB messages too

# Pictures of prototype module

## Front view main screen displaying main info:
* latitude, S:[number of satellites from drone remote unit]
* longitude, VM [vertical speed max in kmph]
* D: distance from tracker unit,  bearing fromk here in short text and compass degrees (this line is enough for locating drone with an external compass)
* UTC time from GPS (any unit last package), A: relative altitude from home

![Image of Screen](images/proto1front1.jpg) 


## Rear view of protoype unit
![Image of Screen](images/proto1back1.jpg) 


## Front view message debug screen displaying last processed sentence of GPS
![Image of Screen](images/proto1front2.jpg) 


## Front view debug screen for counters displaying info:
* [battery voltage]V, S:[numer of Sat from radio]/[number of Sat from local gps]
* RR:[remote radio(drone) serial port number of reconnects], C: [number of CRC errors],  i:[currend serial buffer size]
* GR:[loval GPS serial port number of reconnects], C: [number of CRC errors],  i:[currend serial buffer size]
* A [maximum altitude], D[maximum distance in meters], [millis taken to render current screen]

![Image of Screen](images/proto1front3.jpg) 






