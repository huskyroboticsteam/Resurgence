#!/bin/bash

CAN_BITRATE="${CAN_BITRATE:-125000}"
CAN_DBITRATE="${CAN_DBITRATE:-125000}"

sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

sudo ip link set can0 type can bitrate "${CAN_BITRATE}" dbitrate "${CAN_DBITRATE}" \
     berr-reporting on fd on restart-ms 100
sudo ip link set up can0

if [[ $? != 0 ]]; then
  echo "Error enabling can0 interface!"
fi
