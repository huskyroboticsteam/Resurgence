# PY2020
PY2020 Jetson TX2 Codebase Hindsight

# Project Setup

There are a few setup steps that are necessary before development on the
codebase can be done.

Note: Terminal commands to perform many of these steps can be found in the file
`tests-docker-action/entrypoint.sh`, which is used to automatically set up an
Ubuntu computer in order to run our integration tests.

**Note for Windows users:** It is highly recommended you use Windows Subsystem
for Linux or a VM with a GNU/Linux distribution installed. If you are going to
need USB hardware access (e.g. cameras, LIDAR, etc.) then it is highly
recommended to use a VM, because WSL does not currently support hardware
access.

## Install OpenCV
OpenCV is a computer vision library that's used for the AR tag detection code
and as such is needed to compile the project. We will also need the OpenCV
contrib modules for the AR tag detection code. Either OpenCV 3 or OpenCV 4
should work, but we are developing and testing against OpenCV 4 so you should
try to get that version for best results.

### On GNU/Linux (including Windows Subsystem for Linux)

Open your terminal and execute the following commands. If you are using a
GNU/Linux distribution natively or in a VM, you can often paste with
`CTRL+SHIFT+v`. If you are using Windows Subsystem for Linux, these commands
should be executed in your WSL terminal, and you can paste with right click.

OpenCV and its contrib modules are packaged for Ubuntu GNU/Linux. For other
distributions, check to see if `libopencv` and `libopencv-contrib` are included
in your distribution's package catalog. 

To install, simply run the following commands:

1. `sudo apt update && sudo apt -y upgrade`
2. `sudo apt install libopencv-dev libopencv-contrib-dev`

### On Mac
There is a Homebrew package available for OpenCV on Mac OS. Open the Terminal
and run these commands:
1. `/usr/bin/ruby -e "$(curl -fsSL
   https://raw.githubusercontent.com/Homebrew/install/master/install)"` (If you
   already have Homebrew installed, you can skip this step.)
2. `brew install opencv@4`

## Install Catch2 (for unit testing)
These instructions assume you've followed the steps above to set up OpenCV,
which should also set up some things in the process like the C++ compiler and
CMake, or Homebrew for Mac users.

### On GNU/Linux (including Windows Subsystem for Linux)
Execute the following commands in your terminal:
1. `git clone https://github.com/catchorg/Catch2.git`
2. `cd Catch2`
3. `git checkout v2.13.4`
4. `mkdir build`
5. `cd build`
6. `cmake -H.. -DBUILD_TESTING=OFF`
7. `sudo cmake --build . --target install`

### On Mac
There is a Homebrew package for Catch2 as well. Open the Terminal and run this
command:
`brew install catch2`

## Install URG library (Hokuyo lidar)

Tested on Ubuntu. Not sure how to install on Mac and Windows.

```
git clone https://github.com/andrewbriand/urg_library-1.2.5.git
cd urg_library-1.2.5
make
sudo make install
```

By default on Ubuntu, the USB lidar device will only be accessible to the
root user (e.g. via sudo). To make it accessible to everyone:

```
sudo cp src/lidar/50-usb-hokuyo-lidar.rules /etc/udev/rules.d
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Install SFML and Eigen (navigation simulator)

Follow instructions in `src/simulator/README.md`, or just execute the commands
from `tests-docker-action/entrypoint.sh`.

## Install GPS libraries

On Ubuntu, just run:

```bash
sudo apt-get install gpsd gpsd-clients libgps-dev
```

## Install ROS (planning visualization)

### On Ubuntu (Including Windows Subsystem for Linux)

Make sure you have a locale which supports UTF-8:

```bash
locale  # check for UTF-8
```

If your locale does not support UTF-8, run the following commands:

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

Set up the ROS apt repositories:

```bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
```

Ubuntu 18.04 and 20.04 support different ROS distributions, so replace `<version>` in the commands below with the appropriate version for your system.

18.04: `dashing`  
20.04: `foxy`  

```bash
sudo apt install ros-<version>-ros-base
source /opt/ros/<version>/setup.bash
echo "source /opt/ros/<version>/setup.bash" >> ~/.bashrc
```

### On other systems (not tested)

Follow the instructions here: https://index.ros.org/doc/ros2/Installation/Foxy/

If your system doesn't support ROS Foxy, install ROS Dashing by following the instructions here: https://index.ros.org/doc/ros2/Installation/Dashing/

## Setup Project Repository
This step will require you to have CMake and Git. If you've followed the above
OpenCV setup instructions, you should have these installed already. The
following instructions will assume you have them installed.

These instructions will also assume you are using a Unix-like environment;
GNU/Linux and Mac users should be fine here, Windows users should run these
commands in their WSL terminal.

Enter the home directory (or wherever you would like to put the project files).
`cd ~` (If you would like to put the project somewhere else, replace `~` with
the folder path.)

Clone the repository.
`git clone https://github.com/huskyroboticsteam/PY2020/`

Navigate into the repository.
`cd PY2020`

Clone the submodules
`git submodule init`
`git submodule update`
  
Now, you're ready to build the project.

## Building
Ensure you have CMake installed.
Run
```
mkdir -p build
cd build
cmake ../src
make -j$(nproc)
```
