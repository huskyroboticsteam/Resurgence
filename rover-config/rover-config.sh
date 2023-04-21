#!/usr/bin/env bash
cp -r ./.motion/ /home/$USER
cp ./50-rover-cameras.rules /etc/udev/rules.d/
cp ./50-usb-hokuyo-lidar.rules /etc/udev/rules.d/

# Reloads udev rules instead of having to reboot
sudo udevadm control --reload-rules
