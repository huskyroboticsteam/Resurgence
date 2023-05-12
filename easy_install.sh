#!/bin/bash

sudo apt install -y curl wget
sudo curl -s --compressed "https://huskyroboticsteam.github.io/ubuntu-repo/KEY.gpg" | sudo apt-key add -
sudo wget -P /etc/apt/sources.list.d "https://huskyroboticsteam.github.io/ubuntu-repo/husky_robotics.list"
sudo apt update
sudo apt install -y build-essential unzip gnupg2 lsb-release git \
	cmake libeigen3-dev libopencv-dev libopencv-contrib-dev \
	libwebsocketpp-dev libboost-system-dev gpsd gpsd-clients libgps-dev nlohmann-json3-dev \
	catch2 urg-lidar rplidar ublox-linux hindsight-can=1.3.2 frozen libargparse-dev libavutil-dev \
	libx264-dev

echo "Done installing Resurgence dependencies!"