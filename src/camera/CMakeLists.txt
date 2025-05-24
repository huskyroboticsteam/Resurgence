find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_executable(calibration
  calibration.cpp)

add_executable(panoramic
  ./CameraParams.cpp
  ./Camera.cpp
  ./CameraConfig.cpp
  PanoramicCamera.cpp)

target_link_libraries(calibration ${OpenCV_LIBS})
target_link_libraries(panoramic ${OpenCV_LIBS})
