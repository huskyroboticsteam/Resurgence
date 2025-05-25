find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# FetchContent_Declare(
#     HindsightCAN
#     GIT_REPOSITORY https://github.com/huskyroboticsteam/HindsightCAN.git
#     GIT_TAG cae6622cf6e37889a75dd3fd63676a91a1eef98e
# )

# FetchContent_MakeAvailable(HindsightCAN)

add_executable(calibration
  calibration.cpp)

add_executable(panoramic
  ./CameraParams.cpp
  ./Camera.cpp
  ./CameraConfig.cpp
  PanoramicCamera.cpp)

target_link_libraries(calibration ${OpenCV_LIBS})
target_link_libraries(panoramic ${OpenCV_LIBS} can_interface real_world_interface)

