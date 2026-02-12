#!/bin/bash
# Test script for object detection keyboard control

echo "=== Object Detection Keyboard Control Test ==="
echo ""
echo "This script will test the keyboard control integration."
echo ""
echo "Instructions:"
echo "1. Make sure the model file exists at: build/owlvit-cpp.pt"
echo "2. The simulator should be running"
echo "3. When Rover starts, you'll see keyboard controls"
echo "4. Press 'O' to toggle object detection"
echo "5. Press 'Q' to quit"
echo ""
echo "Starting Rover in 3 seconds..."
sleep 3

cd /home/thomas/hsr/Resurgence/build
./Rover -p arm
