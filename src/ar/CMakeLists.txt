find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_executable(ar_test
  ARTester.cpp
  CameraParams.cpp
  Detector.cpp
  Tag.cpp
  ../ThreadedCapture.cpp
  ../Util.cpp)

set_property(TARGET ar_test PROPERTY CXX_STANDARD 14)
target_link_libraries(ar_test ${OpenCV_LIBS})
target_link_libraries(ar_test Threads::Threads)
