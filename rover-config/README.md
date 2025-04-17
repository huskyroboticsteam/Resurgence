# Rover Config

This directory (`Resurgence/rover-config`) is where configuration files
for the rover's onboard computer should live. If you add any
configuration files to this directory, please list them below with a
description and the location they should be placed on the rover's
filesystem.

| **File/Directory**          | **Location on Rover** | **Description**                                                                |
| -----------------           | --------------------  | --------------                                                                 |
| `50-rover-cameras.rules`    | `/etc/udev/rules.d/`  | `udev` rules for making sure cameras have stable IDs.                          |

# Directions
Run:
```bash
./rover-config.sh
```
Alternatively, one can manually copy the files over.

## Notes

To update `udev` rules, either reboot or execute
```bash
sudo udevadm control --reload-rules
```
after copying the files to `/etc/udev/rules.d`.
