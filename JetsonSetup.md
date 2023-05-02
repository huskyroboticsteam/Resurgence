# Jetson Configuration

# Required Hardware
* Nvidia Jetson TX2 (and included cables/power brick)
* USB Hub
* External Monitor
* Mouse+Keyboard
* HDMI Cable

# Hardware Connection
1. Connect mouse + keyboard to usb hub and connect usb hub to Jetson usb port
2. Connect HDMI cable from external monitor to HDMI port on Jetson
    * USB and HDMI ports are next to each other
3. Power on Jetson using red power button (read board to find)

# Software
1. Ubuntu 18.04 should boot
    * When setting up Ubuntu
        * Your name: husky
        * Computer name: jetson-1
        * Username: husky
        * Password: Ask lead
        * Wifi: Use HuskyRobotics and ask lead for password
2. Ensure current version of Ubuntu is 18.04 by using:
```bash
lsb_release -a # Should see 18.04
```
3. Update to Ubuntu 20.04 (either a window popup will appear or can be done through Software Updater)
4. Once 20.04 is installed and configuration complete confirm version again
```bash
lsb_release -a # Should see 20.04
```
5. Clone Resurgence repository
```bash
git clone https://github.com/huskyroboticsteam/Resugence
```
6. Folow rover-config instructions [here](https://github.com/huskyroboticsteam/Resurgence/tree/master/rover-config)
7. Follow project setup instructions [here](https://github.com/huskyroboticsteam/Resurgence)

# Troubleshooting

1. Ubuntu 18.04 will prompt you to update to 20.04, it will then install all it needs and reboot. This then
   seems like the end of the setup but it usually does not update 20.04 and stays on 18.04. Run the updater to 20.04
   again and after it installs more and reboots again, it should be on 20.04.
2. Update to 20.04 can sometimes fail in slight ways. A common situation is that in the process of updating
   the Jetson will reboot and leave you on a blank screen with a blinking cursor in the top left, if this
   happens:
   1. Switch to a shell using ```CTL + ALT + F3```
   2. Reconfigure dpkg ```sudo dpkg-reconfigure gdm3```
   3. Restart gdm3 ```sudo service gdm3 restart```
   4. Reinstall libappstream4 ```sudo apt install --reinstall libappstream4```
   5. If errors persist ```sudo dpkg --configure -a```
