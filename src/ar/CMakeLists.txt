find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_executable(ar_tester
  ARTester.cpp
  ../camera/Camera.cpp
  Detector.cpp
  MarkerSet.cpp
  MarkerPattern.cpp
  Tag.cpp)

target_link_libraries(ar_tester ${OpenCV_LIBS} opencv_aruco Threads::Threads utils)
