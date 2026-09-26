#!/usr/bin/env bash
sudo cp ./50-rover-cameras.rules /etc/udev/rules.d/

sudo cp ./can-usb-trigger.service /etc/systemd/system/
sudo cp ./99-usb-trigger.rules /etc/udev/rules.d/

# Reloads udev rules instead of having to reboot
sudo udevadm control --reload-rules
sudo udevadm trigger