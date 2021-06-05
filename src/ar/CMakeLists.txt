find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_executable(ar_tester
  ARTester.cpp
  ../camera/CameraParams.cpp
  ../camera/Camera.cpp
  Detector.cpp
  MarkerSet.cpp
  MarkerPattern.cpp
  Tag.cpp)

target_link_libraries(ar_tester ${OpenCV_LIBS} opencv_aruco)
target_link_libraries(ar_tester Threads::Threads)

set_property(TARGET ar_tester PROPERTY CXX_STANDARD 14)
