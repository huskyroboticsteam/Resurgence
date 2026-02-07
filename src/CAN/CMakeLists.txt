cmake_minimum_required(VERSION 3.8)

if(WORLD_INTERFACE STREQUAL "REAL")
	add_executable(FakeCANBoard FakeCANBoard.cpp)
	target_link_libraries(FakeCANBoard can_interface real_world_interface)
endif()
