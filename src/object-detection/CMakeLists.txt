# Set LibTorch path
set(CMAKE_PREFIX_PATH "/home/thomas/libtorch")

# Find dependencies
find_package(Torch REQUIRED)
find_package(OpenCV REQUIRED)

# Create object detection library
add_library(object_detection
    ObjectDetector.cpp
    DetectionResult.cpp
    ModelDownloader.cpp
    read_objects.cpp
)

# Link libraries
target_link_libraries(object_detection
    PUBLIC ${TORCH_LIBRARIES}
    PUBLIC ${OpenCV_LIBS}
    PUBLIC camera
    PUBLIC constants
    PUBLIC world_interface
    PUBLIC loguru
)

# Include directories
target_include_directories(object_detection
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC ${CMAKE_SOURCE_DIR}/camera
    PRIVATE ${OpenCV_INCLUDE_DIRS}
)

# Set C++ standard
set_property(TARGET object_detection PROPERTY CXX_STANDARD 17)

# Test executables
add_executable(object_detection_test ObjectDetectionTest.cpp)
target_link_libraries(object_detection_test 
    object_detection
    control
    -Wl,--no-as-needed  # Force all symbols to be linked
    camera
    constants
    ${OpenCV_LIBS}
    ${TORCH_LIBRARIES}
    -Wl,--as-needed  # Resume normal linking
)
set_property(TARGET object_detection_test PROPERTY CXX_STANDARD 17)

# Camera test executable (for testing with real camera)
add_executable(camera_test CameraTest.cpp)
target_link_libraries(camera_test 
    object_detection
    camera
    constants
    ${OpenCV_LIBS}
    ${TORCH_LIBRARIES}
)
set_property(TARGET camera_test PROPERTY CXX_STANDARD 17)

# RealSense D455 test executable (only when RealSense is available)
if(WITH_REALSENSE)
    add_executable(realsense_test RealSenseTest.cpp)
    target_link_libraries(realsense_test 
        object_detection
        camera
        constants
        ${OpenCV_LIBS}
        ${TORCH_LIBRARIES}
    )
    set_property(TARGET realsense_test PROPERTY CXX_STANDARD 17)
endif()

# Camera calibration tool
add_executable(calibrate_camera CalibrateCamera.cpp)
target_link_libraries(calibrate_camera 
    -Wl,--no-as-needed
    object_detection
    camera
    constants
    utils
    world_interface
    websocket_utils
    rover_common
    -Wl,--as-needed
    ${OpenCV_LIBS}
    ${TORCH_LIBRARIES}
)
set_property(TARGET calibrate_camera PROPERTY CXX_STANDARD 17)
