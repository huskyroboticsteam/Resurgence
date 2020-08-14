# PY2020
PY2020 Jetson TX2 Codebase Hindsight

# Project Setup

There are a few setup steps that are necessary before development on the
codebase can be done.

Note: Terminal commands to perform many of these steps can be found in the file
`tests-docker-action/entrypoint.sh`, which is used to automatically set up an
Ubuntu computer in order to run our integration tests.

## Install OpenCV
OpenCV is a computer vision library that's used for the AR tag detection code
and as such is needed to compile the project.

The setup process takes some time and will use a lot of CPU, but it's fairly
straightforward and should only need to be done once.

We need an extra module (ARUco) for OpenCV for the AR tag detection code which
is not bundled with OpenCV, so you will have to install that module before
building OpenCV. The below instructions will cover this.

### On Windows (with Windows Subsystem for Linux)
Open the WSL terminal and run the following commands (you can paste with right
click):
1. `sudo apt update && sudo apt -y dist-upgrade`
2. `sudo apt install unzip git cmake g++ libgtk2.0-dev pkg-config libavcodec-dev
   libavformat-dev libswscale-dev libtbb2 libtbb-dev libjpeg-dev libpng-dev
   libtiff-dev libdc1394-22-dev`
   
Now, to install the extra modules:

3. `git clone https://github.com/opencv/opencv_contrib`
4. `cd opencv_contrib`
5. `git checkout 3.4.7`
6. `mkdir selected_modules`
7. `cp -r modules/aruco selected_modules/`
8. `cd`

Now, to install and build OpenCV, with the extra modules:

9. `wget https://github.com/opencv/opencv/archive/3.4.7.zip`
10. `unzip 3.4.7.zip`
11. `cd opencv-3.4.7`
12. `mkdir build`
13. `cd build`
14. `cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
    -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/selected_modules ..` 
15. `make -j$(nproc)` (The command `nproc` here will return the number of CPU
   cores on your computer, and will enable `make` to build in parallel, which
   will make the build process go faster.) This step will take a long
   time and use nearly 100% CPU for that entire time, so be warned.
16. `sudo make install`
   
### On GNU/Linux
If you are using a GNU/Linux distribution that supports APT, the instructions
should be the same as for Windows above. If you are using a different
distribution, the only steps that should be different are the first two, which
are installing libraries and utilities OpenCV depends on. In this case you
should install those libraries and utilities with your distribution's package
manager.

### On Mac
There is a Homebrew package available for OpenCV on Mac OS. Open the Terminal
and run these commands:
1. `/usr/bin/ruby -e "$(curl -fsSL
   https://raw.githubusercontent.com/Homebrew/install/master/install)"` (If you
   already have Homebrew installed, you can skip this step.)
2. `brew install opencv@3`

## Install Catch2 (for unit testing)
These instructions assume you've followed the steps above to set up OpenCV,
which should also set up some things in the process like the C++ compiler and
CMake, or Homebrew for Mac users.

### On Windows (with Windows Subsystem for Linux)
Open the WSL terminal and run the following commands:
1. `git clone https://github.com/catchorg/Catch2.git`
2. `cd Catch2`
3. `cmake -Bbuild -H. -DBUILD_TESTING=OFF`
4. `sudo cmake --build build/ --target install`

### On GNU/Linux
The instructions should be the same as for Windows above.

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

## Install SFML and Eigen (navigation simulator)

Follow instructions in `src/simulator/README.md`, or just execute the commands
from `tests-docker-action/entrypoint.sh`.

## Setup CMake
CMake is a tool we use that helps us compile our project with the libraries we
use. This assumes that you have Ubuntu installed and configured.

If you are using Windows, type this into your Ubuntu terminal and hit "return"
to install the necessary files:
`sudo apt install cmake g++ git`

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
