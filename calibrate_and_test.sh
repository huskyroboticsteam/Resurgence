# Camera Calibration and Testing Commands
# Updated for RealSense D455 depth camera support

# === Compilation (REAL mode with RealSense) ===
cd /home/thomas/hsr/Resurgence/build
cmake -DWITH_REALSENSE=ON -DWORLD_INTERFACE=REAL ../src
make realsense_test calibrate_camera camera_test -j4

# === RealSense D455 Mode (Recommended) ===
# Uses depth camera for accurate distance measurement
# Min distance: 52cm, Optimal: 0.6m - 6m
export OWLVIT_MODEL_PATH="/home/thomas/hsr/Resurgence/src/object-detection/owlvit-cpp.pt"
cd /home/thomas/hsr/Resurgence
build/object-detection/realsense_test

# Controls:
#   '1' - Toggle Orange Hammer detection
#   '2' - Toggle Rock Pick detection
#   '3' - Toggle Water Bottle detection
#   '0' - Disable all detection
#   'd' - Toggle depth overlay
#   '+/-' - Adjust confidence threshold
#   'q' - Quit

# === Legacy Local Camera Mode (no depth) ===
# export OWLVIT_MODEL_PATH="/home/thomas/hsr/Resurgence/src/object-detection/owlvit-cpp.pt"
# cd /home/thomas/hsr/Resurgence
# build/object-detection/calibrate_camera 4
# build/object-detection/camera_test 4

# === Simulator Mode ===
# Requires: cmake -DWORLD_INTERFACE=SIMULATOR ../src && make -j4
# export OWLVIT_MODEL_PATH="/home/thomas/hsr/Resurgence/src/object-detection/owlvit-cpp.pt"
# cd /home/thomas/hsr/Resurgence
# build/object-detection/calibrate_camera mast --sim
# build/object-detection/camera_test mast --sim
