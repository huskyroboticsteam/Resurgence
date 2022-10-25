# Resurgence
Main onboard codebase for the Husky Robotics rover.

# Pre-Setup Notes

Our codebase is developed for an NVIDIA Jetson TX2, which runs Ubuntu Linux; as such, much
of our code will be Unix-specific.

> ⚠️ The only supported platform is Ubuntu 20.04 LTS. Other versions/distros/platforms may work, but if you want as easy as possible, go with this version. 

**Windows users:** You should use either [Windows Subsystem for
Linux](https://docs.microsoft.com/en-us/windows/wsl/about) or a VM with a Linux
distribution installed (Ubuntu recommended). Either should work fine. Whichever you use, install either the VM or WSL2 and follow the Linux instructions. As noted above, **use Ubuntu 20.04**.

**Mac users:** We do not make any effort to support Mac systems. You *may* be able to get things working, but if you try you'll be on your own. It's **highly recommended** for Mac users to use a Linux VM.

**From here on out, the installation instructions will assume you are using Ubuntu 20.04 LTS**. Windows users should run commands in either their Linux VM or their WSL
terminal. For Linux users, we'll assume you're running Ubuntu; users of another
distribution may need to change some instructions (e.g. package managers) depending on
your distro.

# Project Setup

## Option 1: Easy Install

Make sure software is up to date (`sudo apt upgrade` is optional) and install required tools:
```bash
sudo apt update
sudo apt install -y git
```

Clone repository:
```bash
cd <place to put repository>
git clone https://github.com/huskyroboticsteam/Resurgence/
```

Install dependencies in one go:
```bash
cd Resurgence
chmod +x easy_install.sh
./easy_install.sh
```

Done! Continue on to the "IDE Setup" section.
## Option 2: Manual Install

### Install System Tools

First, update your software with 
```bash
sudo apt update && sudo apt upgrade
```
Then, to install the tools, run
```bash
sudo apt install build-essential cmake git
```

### Setup Ubuntu repository
The project depends on many third-party libraries (mostly for sensors,
distributed by the sensor manufacturers) that we have packaged in a third-party
Ubuntu package repository. *Before continuing*, you should follow the
instructions at <https://huskyroboticsteam.github.io/ubuntu-repo> to set it
up. Afterwards, it should be fairly trivial to install the dependencies. 

If you are on another Linux distribution, you may still have to build these
dependencies from source (especially if your system does not use the APT package
manager).

### Setup Project Repository

```bash
# Enter the home directory (or wherever you would like to put the project files).
# If you would like to put the project somewhere else, replace '~' with the folder path.
cd ~
# Clone the repository.
git clone https://github.com/huskyroboticsteam/Resurgence/
``` 

### Install HindsightCAN
This is a library developed by Electronics for interfacing with their
motors and sensors over the
[CAN](https://en.wikipedia.org/wiki/CAN_bus) bus; it is needed for packet definitions and utility functions and doesn't actually require support for a physical CAN bus.


Assuming the Ubuntu repository has been set up:
```bash
sudo apt install hindsight-can
```

### Install OpenCV

OpenCV and its contrib modules are packaged for Ubuntu. For other distributions, check to
see if `libopencv` and `libopencv-contrib` are included in your distribution's package
catalog.

To install, run the following command:
```bash
sudo apt install libopencv-dev libopencv-contrib-dev
```

### Install Catch2 (for unit testing)

If you are on Ubuntu, you can use our Ubuntu package repository.
Follow the instructions on <https://huskyroboticsteam.github.io/ubuntu-repo> first, and 
then once you're finished, you can run
```bash
sudo apt install catch2
```

### Install URG library (Hokuyo lidar)

On Ubuntu, make sure you have followed the
[instructions to install our Ubuntu
repo](https://huskyroboticsteam.github.io/ubuntu-repo) and then just run:
```bash
sudo apt install urg-lidar
```

If you need to actually access the hardware (e.g. you're provisioning the jetson, don't do this on your personal machine unless you need to)

```bash
sudo cp src/lidar/50-usb-hokuyo-lidar.rules /etc/udev/rules.d
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Install RPLidar

```bash
sudo apt install rplidar
```

### Install [Eigen](http://eigen.tuxfamily.org)

On Ubuntu, you will be able to install via

```bash
sudo apt-get install libeigen3-dev
```

### Install JSON library

```bash
sudo apt-get install nlohmann-json3-dev
```

### Install WebSocket library

Websocket++ depends on a module from boost, so we'll need to install that too.
```bash
sudo apt install libwebsocketpp-dev libboost-system-dev
```

### Install GPS libraries


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

### Install other libraries

We need the Frozen library for making immutable compile-time constant
containers, and argparse for parsing command-line arguments.

On Ubuntu, make sure you have followed the
[instructions to install our Ubuntu
repo](https://huskyroboticsteam.github.io/ubuntu-repo) and then just run:

```bash
sudo apt-get install frozen libargparse-dev
```

Done!

# IDE Setup

## Native Linux 
If you're running Linux natively, set up your IDE however you like. We recommend [VSCode](https://code.visualstudio.com/).

## WSL2 on Windows
Again, we recommend [VSCode](https://code.visualstudio.com/), which has very good [WSL integration](https://code.visualstudio.com/docs/remote/wsl).

## Linux on VM
You could do one of the following:

a) Develop within the VM, in which case see "Native Linux" above.

b) Create a shared folder (shared between your computer and VM) and clone the project there. Then use an editor to edit the files on your machine and run them within the VM.

c) Set up the VM as an ssh server (pretty easy) and run VSCode on your machine (not the VM) using the [remote development](https://code.visualstudio.com/docs/remote/ssh) feature.

Of these, (c) probably has the most convenient/usable setup, while (a) is probably the least work.

# Running the code
## Set up the build directory
  
Now, you're ready to build the Resurgence project. Return to the Resurgence directory and run:

```
mkdir build
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

Launch the appropriate simulator executable for your platform. Then, run the rover code, using the `p` flag to specify a peripheral:

```bash
./Rover -p {none|arm|science|lidar}
```

The programs can be started in any order, it doesn't matter.

### Switching back to building the real rover code

Since the `Rover` target now builds the simulator rover code instead of the real rover code, we need to reconfigure CMake to build the real rover code again:

```bash
cd build
rm -r *
cmake ../src
make -j Rover
```
