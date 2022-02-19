cmake_minimum_required(VERSION 3.8)

#*  add_executable(Server
#*    Server.cpp)
#*  add_executable(FakeBaseStation
#*    FakeBaseStation.cpp)
add_executable(FakeCANBoard
  FakeCANBoard.cpp
  TestPackets.cpp)
target_link_libraries(FakeCANBoard hindsight_can real_can_interface)

if(WITH_TESTS)
  # Networking Tests
  #*  target_sources(tests PUBLIC
  #*  	#*  ${CMAKE_CURRENT_SOURCE_DIR}/tests.cpp
  #*  	${CMAKE_CURRENT_SOURCE_DIR}/TestPackets.cpp
  #*  	${CMAKE_CURRENT_SOURCE_DIR}/NetworkingStubs.cpp)
endif()
