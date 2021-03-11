find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_executable(ar_tester
  ARTester.cpp
  CameraParams.cpp
  Detector.cpp
  MarkerSet.cpp
  MarkerPattern.cpp
  Tag.cpp
  ../ThreadedCapture.cpp)

add_executable(calibration
  calibration.cpp)

target_link_libraries(ar_tester ${OpenCV_LIBS})
target_link_libraries(ar_tester Threads::Threads)
target_link_libraries(calibration ${OpenCV_LIBS})

set_property(TARGET ar_tester PROPERTY CXX_STANDARD 14)
set_property(TARGET ar_tester PROPERTY EXCLUDE_FROM_ALL true)
set_property(TARGET calibration PROPERTY EXCLUDE_FROM_ALL true)
