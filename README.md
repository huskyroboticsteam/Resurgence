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

## (*Linux users only*) Setup Ubuntu repository
The project depends on many third-party libraries (mostly for sensors,
distributed by the sensor manufacturers) that we have packaged in a third-party
Ubuntu package repository. *Before continuing*, you should follow the
instructions at <https://huskyroboticsteam.github.io/ubuntu-repo> to set it
up. Afterwards, it should be fairly trivial to install the dependencies. 

If you are on another Linux distribution, you may still have to build these
dependencies from source (especially if your system does not use the APT package
manager).

Mac users will unfortunately still have to build them from source, but the
process shouldn't take too long.

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

4. Clone the CAN library submodule (we use git submodules, unfortunately)
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
If you are on Ubuntu, you can use our Ubuntu package repository.
Follow the instructions on <https://huskyroboticsteam.github.io/ubuntu-repo> first, and 
then once you're finished, you can run
```bash
sudo apt install catch2
```

### On Mac
There is a Homebrew package for Catch2 as well. Open the Terminal and run this
command:
```bash
brew install catch2
```

## Install URG library (Hokuyo lidar)

~~This is **optional** unless you are connecting to our hardware lidar
sensor.~~ (not yet!)

### Linux
On Ubuntu, make sure you have followed the
[instructions to install our Ubuntu
repo](https://huskyroboticsteam.github.io/ubuntu-repo) and then just run:
```bash
sudo apt install urg-lidar
```


### Mac
Unfortunately on Mac, you will have to build the library from source. These instructions
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


(TODO: actually include udev rules) ~~By default on Ubuntu, the USB lidar device
will only be accessible to the root user (e.g. via sudo). To make it accessible
to everyone:~~

```
sudo cp src/lidar/50-usb-hokuyo-lidar.rules /etc/udev/rules.d
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Install [Eigen](http://eigen.tuxfamily.org)

### Linux
On Ubuntu, you will be able to install via

```bash
sudo apt-get install libeigen3-dev
```

### Mac
On Mac, you will be able to install via
```bash
brew install eigen
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

## Install WebSocket library

### Ubuntu 20.04
Websocket++ depends on a module from boost, so we'll need to install that too.
```bash
sudo apt install libwebsocketpp-dev libboost-system-dev
```

## Install GPS libraries

~~This is **optional** unless you are connecting to our hardware GPS
sensor.~~ (not yet!** This is required to build for now.

### Linux

On Ubuntu, make sure you have followed the
[instructions to install our Ubuntu
repo](https://huskyroboticsteam.github.io/ubuntu-repo) and then just run:

```bash
sudo apt-get install ublox-linux gpsd gpsd-clients libgps-dev
```
(**NOTE**: `gpsd` is still required for legacy purposes, but will likely be
phased out when we completely switch to the new Ublox GPS hardware, so this
command installs `ublox-linux` as well to interact with that. Even `ublox-linux`
is a temporary solution; Electronics plans to integrate the GPS into a board
which we will query over CAN.)

### Mac

On Mac, you are able to install `gpsd` via
```bash
brew install gpsd
```
but you will have to build `ublox-linux` from source. These instructions
can also be found in the
[huskyrobotics/ublox-linux](https://github.com/huskyroboticsteam/ublox-linux)
repository's README.
```bash
# Clone repository (--recursive is important! Build won't work without submodules)
git clone --recursive https://github.com/huskyroboticsteam/ublox-linux.git
# Enter cloned directory
cd ublox-linux
# Make a build directory to hold compiled code
mkdir -p build
# Enter build directory
cd build
# Configure CMake
cmake ..
# Build/Install Ublox Linux library
sudo make install
# Navigate back to working directory
cd ../..
```

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

Otherwise you can specify just the specific executable you would like to run, e.g. `make Rover`. (You can still use the `-j` flag to parallelize the build.)

To run our unit tests, run `make tests` and then execute `./tests`.

## Running the Simulator

You can download the latest simulator build from the [simulator releases tab](https://github.com/huskyroboticsteam/Simulator/releases/latest).

### Building the rover code to work with the simulator

The simulator does not have its own executable. Instead, you must configure the CMake variables and build the `Rover` target:

```bash
cd build
cmake ../src -DWITH_CAN=OFF -DWITH_GPS=OFF -DWITH_LIDAR=OFF -DWITH_TESTS=OFF -DWORLD_INTERFACE=SIMULATOR
make -j Rover
```

Note that (for now) unit tests cannot be run when configured to build the simulator rover code.

### Launching the simulator

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
