find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_executable(calibration
  calibration.cpp)

target_link_libraries(calibration ${OpenCV_LIBS})

if(WORLD_INTERFACE STREQUAL "REAL")
  add_executable(panoramic
    CameraParams.cpp
    Camera.cpp
    CameraConfig.cpp
    H264PassthroughSource.cpp
    PanoramicCamera.cpp
  )

  target_include_directories(panoramic PRIVATE
    ${GSTREAMER_INCLUDE_DIRS}
  )

  target_link_libraries(panoramic
    ${OpenCV_LIBS}
    ${GSTREAMER_LIBRARIES}
    can_interface
    real_world_interface
  )
endif ()