#!/bin/bash

sudo apt install -y curl wget

# Transfer key
sudo mkdir -p /etc/apt/keyrings/
sudo wget /etc/apt/keyrings/ "https://huskyroboticsteam.github.io/ubuntu-repo/KEY.gpg"
sudo gpg --no-default-keyring --keyring ./temp-keyring.gpg --import /etc/apt/keyrings/KEY.gpg
sudo gpg --no-default-keyring --keyring ./temp-keyring.gpg --export --output /etc/apt/keyrings/KEY.gpg
sudo rm temp-keyring.gpg

sudo wget -P /etc/apt/sources.list.d "https://huskyroboticsteam.github.io/ubuntu-repo/husky_robotics.list"
./update_deps.sh

echo "Done installing Resurgence dependencies!"
