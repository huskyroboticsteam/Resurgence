# Rover Config

This directory (`PY2020/rover-config`) is where configuration files
for the rover's onboard computer should live. If you add any
configuration files to this directory, please list them below with a
description and the location they should be placed on the rover's
filesystem.

| **File/Directory**          | **Location on Rover** | **Description**                                                                |
| -----------------           | --------------------  | --------------                                                                 |
| `.motion/`                  | `/home/$USER/`        | Configuration files for Motion (currently handles our video streaming.)        |
| `50-usb-hokuyo-lidar.rules` | `/etc/udev/rules.d/`  | `udev` rules for making sure we have permissions to access the lidar over USB. |
| `50-rover-cameras.rules`    | `/etc/udev/rules.d/`  | `udev` rules for making sure cameras have stable IDs.                          |

## Notes

To update `udev` rules, either reboot or execute
```bash
sudo udevadm control --reload-rules
```
after copying the files to `/etc/udev/rules.d`.
