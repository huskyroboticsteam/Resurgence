#!/bin/bash

CAN_BITRATE=125000
CAN_DBITRATE=500000
GPS_PATH="/dev/ttyUSB0"

sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

sudo ip link set can0 type can bitrate "${CAN_BITRATE}" dbitrate "${CAN_DBITRATE}" \
     berr-reporting on fd on
sudo ip link set can1 type can bitrate "${CAN_BITRATE}" dbitrate "${CAN_DBITRATE}" \
     berr-reporting on fd on
sudo ip link set up can0
sudo ip link set up can1

if [[ $? != 0 ]]; then
  echo "Could not enable hardware CAN interface; using virtual CAN instead."
  sudo ip link add type vcan
  sudo ip link set dev vcan0 up
fi

sudo killall gpsd
sudo service gpsd start
sudo service gpsd stop
sudo systemctl stop gpsd.socket
if [[ -e "${GPS_PATH}" ]]; then
  sudo gpsd -n "${GPS_PATH}"
else
  echo "No device connected at ${GPS_PATH}; gpsd not started!"
fi
