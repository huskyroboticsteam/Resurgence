find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_executable(ar_tester
  ARTester.cpp
  ../camera/CameraParams.cpp
  ../camera/Camera.cpp
  ../camera/CameraConfig.cpp
  Detector.cpp
  MarkerSet.cpp
  MarkerPattern.cpp
  Tag.cpp)

target_link_libraries(ar_tester ${OpenCV_LIBS} opencv_aruco Threads::Threads utils)

# Simple test to get camera from simulator
add_executable(test_sim_camera
  TestSimCamera.cpp
  ../Globals.cpp)

target_link_libraries(test_sim_camera ${OpenCV_LIBS} Threads::Threads utils world_interface websocket_utils)

# Simulator AR Tester - for testing AR detection with simulator camera
add_executable(sim_ar_tester
  SimulatorARTester.cpp
  ../camera/CameraParams.cpp
  ../Globals.cpp
  Detector.cpp
  MarkerSet.cpp
  MarkerPattern.cpp
  Tag.cpp)

target_link_libraries(sim_ar_tester ${OpenCV_LIBS} opencv_aruco Threads::Threads utils world_interface websocket_utils)
