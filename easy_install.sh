#!/bin/bash

sudo apt install -y curl wget
sudo curl -s --compressed "https://huskyroboticsteam.github.io/ubuntu-repo/KEY.gpg" | sudo apt-key add -
sudo wget -P /etc/apt/sources.list.d "https://huskyroboticsteam.github.io/ubuntu-repo/husky_robotics.list"
./update_deps.sh

echo "Done installing Resurgence dependencies!"