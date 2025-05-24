find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

FetchContent_Declare(
    HindsightCAN
    GIT_REPOSITORY https://github.com/huskyroboticsteam/HindsightCAN.git
    GIT_TAG 6ae5db46856cd1f745ad5e1745a6983f4fc372c1
)

add_executable(calibration
  calibration.cpp)

add_executable(panoramic
  ./CameraParams.cpp
  ./Camera.cpp
  ./CameraConfig.cpp
  PanoramicCamera.cpp)

target_link_libraries(calibration ${OpenCV_LIBS} can_interface real_world_interface)
target_link_libraries(panoramic ${OpenCV_LIBS})

