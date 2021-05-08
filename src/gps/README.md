
# Overview

To get GPS data, you need to (install and) run the gpsd daemon. This runs in the background as a separate process, after which you can query for GPS data using a C++ interface.

# Information about BU-353S4 sensor

Manufacturer website is not very informative:
https://www.globalsat.com.tw/en/product-199952/Cable-GPS-with-USB-interface-SiRF-Star-IV-BU-353S4.html
https://www.globalsat.com.tw/ftp/download/GMouse_Win_UsersGuide-V1.0.pdf

The status LED for the GPS is very small, on the side of the sensor. It should turn on (solid) when you plug it into a computer.

The GPS has a magnet on the bottom---be careful.

You can check that we can receive data from the sensor as follows. Replace /dev/ttyUSB0 with whichever /dev is associated with the GPS. (You can figure this out by unplugging it / replugging it and comparing the output of `ls /dev`.)

sudo stty -F /dev/ttyUSB0 ispeed 4800
sudo cat /dev/ttyUSB0

You should see lines looking like this:

$GPRMC,011231.147,V,,,,,,,111220,,,N*4E
$GPGGA,011232.147,,,,,0,00,,,M,0.0,M,,0000*57
$GPGSA,M,1,,,,,,,,,,,,,,,*12

# Installing and running gpsd

sudo apt install gpsd gpsd-clients

If you have issues, this web page seems useful:
https://gpsd.gitlab.io/gpsd/installation.html

Start gpsd:
gpsd -D 5 -N -n /dev/ttyUSB0

Note:
For some reason this sometimes fails with "can't bind IPv4 port gpsd, Address already in use".
In this case, you might be able to fix this with:
  sudo service gpsd start
  sudo service gpsd stop

# Human readable output

I couldn't get `xgps` to work, but `cgps` seems to run fine. While waiting for a GPS fix, you should see JSON-style messages scrolling through the console.

I was able to get a GPS fix (blinking red LED) after 4 or 5 minutes outside. At that point, cgps should show "Status: 3D FIX" and latitute/longitude/altitude information.

# Machine readable output

sudo apt install libgps-dev

Compile the test program gpsd_test.cpp using
  g++ gpsd_test.cpp -l gps

Or just run `make gpsd_test` from the usual build directory.

