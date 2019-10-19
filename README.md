# PY2020
PY2020 Jetson TX2 Codebase [Untitled Rover]

# Setup CMake
CMake is a tool we use that helps us compile our project with the libraries we use. This assumes that you have Ubuntu installed and configured.

If you are using Windows, type this into your Ubuntu terminal and hit "return" to install the necessary files:
  ```sudo apt install cmake g++ git```

If you are using MacOS, type this instead.
```brew install cmake git```

The terminal will prompt you for a password and confirmation; proceed.

When done, clone the repository.
  ```git clone https://github.com/huskyroboticsteam/PY2020/```

Navigate into the repository.
  ```cd PY2020```
  
Now, you're ready to build your make file.

## Building
Ensure you have CMake installed.
Run
```
mkdir build
cd build
cmake ../src
make
```
