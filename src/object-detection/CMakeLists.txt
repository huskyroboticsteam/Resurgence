# Set LibTorch path
set(CMAKE_PREFIX_PATH "/home/thomas/libtorch")

# Find dependencies
find_package(Torch REQUIRED)
find_package(OpenCV REQUIRED)

# Create object detection library
add_library(object_detection
    ObjectDetector.cpp
    DetectionResult.cpp
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

# Test executable
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
