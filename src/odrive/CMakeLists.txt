cmake_minimum_required(VERSION 3.13)

add_library(odrive_interface STATIC Odrive.cpp)
target_link_libraries(odrive_interface utils)

add_executable(FakeODrive FakeODrive.cpp)
target_link_libraries(FakeODrive
  odrive_interface
  nlohmann_json::nlohmann_json
)
