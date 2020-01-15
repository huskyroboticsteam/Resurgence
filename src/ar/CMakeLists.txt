find_package(OpenCV REQUIRED)
add_executable(ar_test
  ARTester.cpp
  Detector.cpp
  Tag.cpp
  ../Util.cpp)

target_link_libraries(ar_test ${OpenCV_LIBS})

set_property(TARGET ar_test PROPERTY CXX_STANDARD 14)
