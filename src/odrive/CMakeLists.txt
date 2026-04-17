cmake_minimum_required(VERSION 3.8)

if(WORLD_INTERFACE STREQUAL "REAL")
add_executable(FakeODrive FakeODrive.cpp)
target_link_libraries(FakeODrive
  can_interface
  ${BOOST_LIBRARIES}
  nlohmann_json::nlohmann_json
  CAN26
)
endif()
