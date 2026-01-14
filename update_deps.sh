#!/bin/bash

sudo apt-get update
sudo apt-get install -y build-essential unzip gnupg2 lsb-release git \
	cmake libeigen3-dev libopencv-dev=4.5.4+dfsg-9ubuntu4 libopencv-contrib-dev \
	libwebsocketpp-dev libboost-system-dev gpsd gpsd-clients libgps-dev nlohmann-json3-dev \
	catch2 ublox-linux frozen libargparse-dev libavutil-dev libx264-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
