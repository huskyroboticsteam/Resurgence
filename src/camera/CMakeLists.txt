find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_executable(camera_tester
  CameraTester.cpp
  CameraParams.cpp
  Camera.cpp)

target_link_libraries(camera_tester ${OpenCV_LIBS})
target_link_libraries(camera_tester Threads::Threads)
