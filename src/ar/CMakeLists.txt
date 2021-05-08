find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_executable(ar_tester
  ARTester.cpp
  ../camera/CameraParams.cpp
  Detector.cpp
  MarkerSet.cpp
  MarkerPattern.cpp
  Tag.cpp
  # TODO: remove/change this once on the new camera system
  ../ThreadedCapture.cpp)

target_link_libraries(ar_tester ${OpenCV_LIBS})
target_link_libraries(ar_tester Threads::Threads)

set_property(TARGET ar_tester PROPERTY CXX_STANDARD 14)
set_property(TARGET ar_tester PROPERTY EXCLUDE_FROM_ALL true)
