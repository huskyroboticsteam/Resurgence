# Resurgence
Main onboard codebase for the Husky Robotics rover.

# Updating Dependencies

Some of our dependencies are team-managed, including the CAN library and the H264Encoder. Leads can use the [ubuntu-repo](https://github.com/huskyroboticsteam/ubuntu-repo) to create new builds of these dependencies when they are updated.

**IMPORTANT:** When a dependency is updated, remember to update the required version number in [CMakeLists.txt](src/CMakeLists.txt) as well as in the [CI script](.github/workflows/ccpp.yml).

# Pre-Setup Notes

Our codebase is developed for an NVIDIA Jetson TX2, which runs Ubuntu Linux; as such, much
of our code will be Unix-specific.

> ⚠️ The only supported platform is Ubuntu 20.04 LTS. Other versions/distros/platforms may work, but if you want as easy as possible, go with this version. 

**Windows users:** You should use either [Windows Subsystem for
Linux](https://docs.microsoft.com/en-us/windows/wsl/about) or a VM with a Linux
distribution installed (Ubuntu recommended). Either should work fine. Whichever you use, install either the VM or WSL2 and follow the Linux instructions. As noted above, **use Ubuntu 20.04**.

**Mac users:** We do not make any effort to support Mac systems. You *may* be able to get things working, but if you try you'll be on your own. It's **highly recommended** for Mac users to use a Linux VM.

> ⚠️ For Mac users, we recommend running a Ubuntu virtual machine via [UTM](https://mac.getutm.app/). After installing the app, set up your VM using [UTM's Ubuntu image](https://mac.getutm.app/gallery/ubuntu-20-04). Please note that UTM only supports the latest version of Ubuntu (20.04). Since the codebase is based on an older version of Ubuntu, there may be package errors that you need to manually resolve. For the sake of convenience, we also recommend SSHing into UTM via VSCode's Remote SSH feature. [More info can be found here](https://arteen.linux.ucla.edu/ssh-into-utm-vm.html). 

**From here on out, the installation instructions will assume you are using Ubuntu 20.04 LTS**. Windows users should run commands in either their Linux VM or their WSL
terminal. For Linux users, we'll assume you're running Ubuntu; users of another
distribution may need to change some instructions (e.g. package managers) depending on
your distro.

# Project Setup

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
./easy_install.sh
```

Done! Continue on to the "IDE Setup" section.

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

## Formatting the code with clang-format

Run clang-format on every edited file. **Github will block your merge if you try to merge code that has not been clang-format'ed correctly!!!**

```
clang-format -i /path/to/file/<FILENAME>/
```
For more information about clang-format, [please see the documentation](https://clang.llvm.org/docs/ClangFormatStyleOptions.html). 

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
cmake ../src -DWORLD_INTERFACE=SIMULATOR
make -j Rover
```

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
cmake ../src -DWORLD_INTERFACE=REAL
make -j Rover
```
