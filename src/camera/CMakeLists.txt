find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_executable(calibration
  calibration.cpp)

target_link_libraries(calibration ${OpenCV_LIBS})
