# Resurgence
Main onboard codebase for the Husky Robotics 2021-2022 rover Resurgence.

# Project Setup
### Notes for users of non-Linux operating systems

Our codebase is developed for an NVIDIA Jetson TX2, which runs Ubuntu Linux; as such, much
of our code will be Unix-specific. Mac is Unix-like, so most things should work on Mac
(except for a few Linux-specific bits) but Windows will not be compatible as it uses a
completely different API.

**Windows users:** You should use either [Windows Subsystem for
Linux](https://docs.microsoft.com/en-us/windows/wsl/about) or a VM with a Linux
distribution installed (Ubuntu recommended). **In most cases, it's highly recommended you
use a VM**, since WSL does not support USB hardware access (for cameras, LiDAR, etc.,
**and only supports graphics on Windows 11**. If you are going to need either of these,
**please use a VM.**

**Mac users:** Mac does not support
[CAN](https://en.wikipedia.org/wiki/CAN_bus), the protocol which we
use to communicate with hardware made by the Electronics team. If you
are doing any low-level hardware integration (e.g. controlling motors
or communicating with any Electronics board) then **please use a VM**
to develop the code (so that Linux kernel headers are present); you
may need to use the Jetson anyway to test the CAN code since usually
only embedded computers have CAN hardware. Otherwise, Mac should
mostly work out of the box (_TODO: verify this_) although the
installation instructions are different.

**From here on out, the installation instructions will assume you are using Linux or
Mac**. Windows users should run commands in either their Linux VM or their WSL
terminal. For Linux users, we'll assume you're running Ubuntu; users of another
distribution may need to change some instructions (e.g. package managers) depending on
your distro. If you are using a GNU/Linux distribution natively or in a VM, you can often
paste with `CTRL+SHIFT+v`. If you are using Windows Subsystem for Linux, these commands
should be executed in your WSL terminal, and you can paste with right click. Mac users
should be able to paste with `Command+v`.

## Install System Tools
For further steps (and development in general) you'll need:
- Git
- C++ compiler
- CMake (the build system we use; takes care of finding libraries, setting appropriate
  compiler options, etc.)

### Linux
First, update your software with 
```bash
sudo apt update && sudo apt upgrade
```
Then, to install the tools, run
```bash
sudo apt install build-essential cmake git
```

### Mac
First, you'll need to get the [Homebrew](https://brew.sh) package manager
installed. To do this, run the following command in a terminal:
```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

Then, to install the tools, run
```bash
brew install cmake
```
	
## Setup Project Repository

1. Enter the home directory (or wherever you would like to put the project files).
`cd ~` (If you would like to put the project somewhere else, replace `~` with
the folder path.)

2. Clone the repository.
```bash
git clone https://github.com/huskyroboticsteam/Resurgence/
```

3. Navigate into the repository.
```bash
cd Resurgence
```

4. Clone the 2D simulator and CAN library submodules (we use git submodules, unfortunately)
```bash
git submodule init
git submodule update
```

5. Finally, navigate back to the parent directory:
```bash
cd ..
```

## Install OpenCV

OpenCV takes a while to install ~~and is **optional** unless you're running
computer vision code.~~ (not yet!)

OpenCV is a computer vision library that's used for our computer vision code, including
the AR tag detection. We will also need its extra contributed ("contrib") modules for its
ARUco implementation. **You must install OpenCV 4, preferably at least OpenCV 4.1.0**
(although most of our code will be built against 4.2.0, since that is what is packaged in
Ubuntu's repositories, any 4.X.X version should theoretically work).

### Linux

OpenCV and its contrib modules are packaged for Ubuntu. For other distributions, check to
see if `libopencv` and `libopencv-contrib` are included in your distribution's package
catalog.

To install, run the following command:
```bash
sudo apt install libopencv-dev libopencv-contrib-dev
```

### On Mac
There is a Homebrew package available for OpenCV on Mac OS; run this command to install it:
```bash
brew install opencv@4
```

## Install Catch2 (for unit testing)
### Linux
1. Navigate to a directory where you would like to put Catch2: `cd ~` (Again, we're using
the home directory `~` here but you can use whatever you'd like.)
1. Clone Catch2: `git clone https://github.com/catchorg/Catch2.git`
2. Enter the cloned directory: `cd Catch2`
3. Check out the correct version: `git checkout v2.13.7`
4. Create and enter a directory to put the compiled code into: `mkdir build && cd build`
6. Configure CMake: `cmake -DBUILD_TESTING=OFF ..`
7. Build/install Catch2: `sudo cmake --build . --target install`
8. Navigate back to the directory from Step 1: `cd ../..`

### On Mac
There is a Homebrew package for Catch2 as well. Open the Terminal and run this
command:
`brew install catch2`

## Install URG library (Hokuyo lidar)

~~This is **optional** unless you are connecting to our hardware lidar
sensor.~~ (not yet!)
The same instructions will work for Linux and Mac. These instructions
can also be found in the
[huskyrobotics/urg-lidar](https://github.com/huskyroboticsteam/urg-lidar)
repository's README.
```bash
# Clone repository
git clone https://github.com/huskyroboticsteam/urg-lidar.git
# Enter cloned directory
cd urg-lidar
# Make a build directory to hold compiled code
mkdir -p build
# Enter build directory
cd build
# Configure CMake
cmake ..
# Build/Install URG lidar library
sudo make install
# Navigate back to working directory
cd ../..
```


(TODO: actually include udev rules) ~~By default on Ubuntu, the USB
lidar device will only be accessible to the root user (e.g. via
sudo). To make it accessible to everyone:~~

```
sudo cp src/lidar/50-usb-hokuyo-lidar.rules /etc/udev/rules.d
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Install SFML and Eigen (navigation simulator)

 * [Eigen](http://eigen.tuxfamily.org)
 * [SFML](https://www.sfml-dev.org/tutorials/2.5/)

On Ubuntu, you might be able to install these via

```
sudo apt-get install libeigen3-dev
sudo apt-get install libsfml-dev
```

## Install JSON library

### Ubuntu 20.04
```bash
sudo apt-get install nlohmann-json3-dev
```

### Mac
```bash
brew install nlohmann-json
```

## Install GPS libraries

This is **optional** unless you are connecting to our hardware GPS
sensor.

On Ubuntu, just run:

```bash
sudo apt-get install gpsd gpsd-clients libgps-dev
```

## Install ROS (planning visualization)

### On Linux

Make sure you have a locale which supports UTF-8:

```bash
locale  # check for UTF-8
```

If the output of the previous command *does not* say `UTF-8`, then
your locale does not support UTF-8, and you should run the following
commands. **If it does say `UTF-8`, you can skip this step; most users
should be able to skip this step.**

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

Ubuntu 18.04 and 20.04 support different ROS distributions, so replace
`<version>` in the commands below with the appropriate version for
your system. **Please note you shouldn't include the `<>` characters.**

18.04: `dashing`
20.04: `foxy`

```bash
# Replace "<version>" with "dashing"/"foxy"; don't include <>!
sudo apt install ros-<version>-ros-base
source /opt/ros/<version>/setup.bash
echo "source /opt/ros/<version>/setup.bash" >> ~/.bashrc
```

### On other systems (not tested)

Follow the instructions here: https://index.ros.org/doc/ros2/Installation/Foxy/

If your system doesn't support ROS Foxy, install ROS Dashing by following the instructions here: https://index.ros.org/doc/ros2/Installation/Dashing/

## Set up the build directory
  
Now, you're ready to build the Resurgence project. Return to the Resurgence directory and run:

```
mkdir -p build
cd build
cmake ../src
```

## Compile the code

To build all of our executables (requires having all optional dependencies installed, e.g. OpenCV and URG), run

`make -j$(nproc)`

Otherwise you can specify just the specific executable you would like to run, e.g. `make RoverSim`. (You can still use the `-j` flag to parallelize the build.)

To run our unit tests, run `make tests` and then execute `./tests`.

## Running the Simulator

You can download the latest simulator build from the [simulator releases tab](https://github.com/huskyroboticsteam/Simulator/releases/latest).

### Building the rover code to work with the simulator

The simulator does not have its own executable. Instead, you must configure the CMake variables and build the `Rover` target:

```bash
cd build
cmake ../src -DWITH_CAN=OFF -DWITH_GPS=OFF -DWITH_LIDAR=OFF -DWITH_TESTS=OFF -DWORLD_INTERFACE=SIMULATOR3D
make -j Rover
```

Note that (for now) unit tests cannot be run when configured to build the simulator rover code.

### Running the simulator

Launch the appropriate simulator executable for your platform. Then, run the rover code:

```bash
./Rover
```

The programs can be started in any order, it doesn't matter.

### Switching back to building the real rover code

Since the `Rover` target now builds the simulator rover code instead of the real rover code, we need to reconfigure CMake to build the real rover code again:

```bash
cd build
rm -rf *
cmake ../src
make -j Rover
```
