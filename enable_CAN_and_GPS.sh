#!/bin/bash

CAN_BITRATE=125000
CAN_DBITRATE=125000
GPS_PATH="/dev/ttyUSB0"

sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

sudo ip link set can0 type can bitrate "${CAN_BITRATE}" dbitrate "${CAN_DBITRATE}" \
     berr-reporting on fd on restart-ms 100
sudo ip link set up can0

if [[ $? != 0 ]]; then
  echo "Error enabling can0 interface!"
fi

# For some reason, on the rover, the gpsd that starts on boot does not
# seem to properly publish information from our GPS.
# Hence, we first make sure that it is not running, and then we restart it.
sudo killall gpsd
sudo systemctl stop gpsd.socket
if [[ -e "${GPS_PATH}" ]]; then
  sudo gpsd -n "${GPS_PATH}"
else
  # This code also ensures we do *not* start gpsd if there is no GPS connected,
  # which allows for some performance optimizations. (For some reason, in the
  # C++ gps library, it is easy to check whether gpsd is running, but it is hard
  # to check whether gpsd is actually connected to any GPS hardware.)
  echo "No device connected at ${GPS_PATH}; gpsd not started!"
fi
