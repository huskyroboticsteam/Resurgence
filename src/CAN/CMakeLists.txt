cmake_minimum_required(VERSION 3.8)

add_executable(FakeCANBoard
  FakeCANBoard.cpp
  TestPackets.cpp)
target_link_libraries(FakeCANBoard can_interface)

add_executable(CANTest CANTest.cpp)
target_link_libraries(CANTest can_interface)

add_executable(TestIMU TestIMU.cpp)
target_link_libraries(TestIMU can_interface)

if(WITH_TESTS)
  # Networking Tests
  target_sources(tests PUBLIC
 # add test code here 
  )
endif()
