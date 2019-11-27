# PY2020
PY2020 Jetson TX2 Codebase [Untitled Rover]

# Project Setup

There are a few setup steps that are necessary before development on the
codebase can be done.

## Install OpenCV
OpenCV is a computer vision library that's used for the AR tag detection code
and as such is needed to compile the project.

The setup process takes some time and will use a lot of CPU, but it's fairly
straightforward and should only need to be done once.

### On Windows (with Windows Subsystem for Linux)
Open the WSL terminal and run the following commands (you can paste with right
click):
1. `sudo apt update && sudo apt -y dist-upgrade`
2. `sudo apt install unzip git cmake g++ libgtk2.0-dev pkg-config libavcodec-dev
   libavformat-dev libswscale-dev libtbb2 libtbb-dev libjpeg-dev libpng-dev
   libtiff-dev libdc1394-22-dev`
3. `wget https://github.com/opencv/opencv/archive/3.4.7.zip`
4. `unzip 3.4.7.zip`
5. `cd opencv-3.4.7`
6. `cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -Bbuild
   .`
7. `sudo cmake --build build --target install -- -j8` (**Replace the number 8 here
   with the number of CPU cores on your computer.**)
   
### On GNU/Linux
If you are using a GNU/Linux distribution that supports APT, the instructions
should be the same as for Windows above. If you are using a different
distribution, the only steps that should be different are the first two, which
are installing libraries OpenCV depends on. In this case you should install
those libraries with your distribution's package manager.

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

## Setup CMake
CMake is a tool we use that helps us compile our project with the libraries we
use. This assumes that you have Ubuntu installed and configured.

If you are using Windows, type this into your Ubuntu terminal and hit "return"
to install the necessary files:
`sudo apt install cmake g++ git`

If you are using MacOS, type this instead.
`brew install cmake git`

The terminal will prompt you for a password and confirmation; proceed.

When done, clone the repository.
`git clone https://github.com/huskyroboticsteam/PY2020/`

Navigate into the repository.
`cd PY2020`
  
Now, you're ready to build the project.

## Building
Ensure you have CMake installed.
Run
<<<<<<< HEAD
=======
```
cmake -Bbuild ./src
cmake --build build
```
>>>>>>> 2d605eeae0c1cb9b15455f8044c5d264bbbad946
