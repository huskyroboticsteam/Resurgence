find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_executable(camera_tester
  CameraTester.cpp
  CameraParams.cpp
  CameraConfig.cpp
  Camera.cpp)

add_executable(calibration
  calibration.cpp)

target_link_libraries(calibration ${OpenCV_LIBS})
target_link_libraries(camera_tester ${OpenCV_LIBS})
target_link_libraries(camera_tester Threads::Threads)
