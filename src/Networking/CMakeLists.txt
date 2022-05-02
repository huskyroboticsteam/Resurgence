cmake_minimum_required(VERSION 3.8)

add_executable(Server
  Server.cpp)
add_executable(FakeBaseStation
  FakeBaseStation.cpp)

if(WITH_TESTS)
  # Networking Tests
  target_sources(tests PUBLIC
	#*  ${CMAKE_CURRENT_SOURCE_DIR}/tests.cpp
	)
endif()
