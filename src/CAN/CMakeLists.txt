cmake_minimum_required(VERSION 3.8)

add_executable(FakeCANBoard
  FakeCANBoard.cpp
  TestPackets.cpp)
target_link_libraries(FakeCANBoard can_interface Threads::Threads)

if(WITH_TESTS)
  # Networking Tests
  target_sources(tests PUBLIC
 # add test code here 
  )
endif()
