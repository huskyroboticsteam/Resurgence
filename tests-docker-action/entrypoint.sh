#!/bin/bash
git submodule update --init --recursive
source /opt/ros/foxy/setup.bash

apt-get update
apt-get install -y gpsd gpsd-clients libgps-dev

mkdir ./build
cd build
cmake ../src
cd ..
cmake --build ./build -j$(nproc)
./build/tests
