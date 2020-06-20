cmake_minimum_required(VERSION 3.2)
set(CMAKE_CXX_STANDARD 11) # For the json library
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(Rover LANGUAGES CXX C)

enable_testing()

find_package(Catch2 REQUIRED)

find_package(OpenCV REQUIRED)

add_definitions(-DCHIP_TYPE=CHIP_TYPE_TEMPLATE)

enable_testing()

find_package(Catch2 REQUIRED)

add_executable(Rover
  Rover.cpp
  CommandLineOptions.cpp
  Globals.cpp
  Networking/Network.cpp
  Networking/CANUtils.cpp
  Networking/ParseBaseStation.cpp
  Networking/IK.cpp
  Util.cpp
  mapping/EnvMap.cpp
  Networking/ParseCAN.cpp
  Networking/log.cpp
  HindsightCAN/CANPacket.c
  HindsightCAN/CANCommon.c
  HindsightCAN/PortTemplate.c
  Util.cpp
  mapping/EnvMap.cpp)

add_executable(RoverNoCAN
  Rover.cpp
  CommandLineOptions.cpp
  Globals.cpp
  Networking/Network.cpp
  Networking/CANStubs.cpp
  Networking/ParseBaseStation.cpp
  Networking/IK.cpp
  Networking/log.cpp
  Util.cpp
  mapping/EnvMap.cpp
  Networking/ParseCAN.cpp
  HindsightCAN/CANPacket.c
  HindsightCAN/CANCommon.c
  HindsightCAN/PortTemplate.c
  Util.cpp
  mapping/EnvMap.cpp)

add_executable(tests
  Tests.cpp
  Util.cpp
  # AR Detection tests
  ar/TagUnitTests.cpp
  ar/Tag.cpp
  # CAN tests
  Networking/tests.cpp
  Networking/TestPackets.cpp
  Networking/log.cpp
  CommandLineOptions.cpp
  Globals.cpp
  Networking/ParseCAN.cpp
  Networking/ParseBaseStation.cpp
  Networking/IK.cpp
  Networking/NetworkingStubs.cpp
  HindsightCAN/CANPacket.c
  HindsightCAN/CANCommon.c
  HindsightCAN/PortTemplate.c
  # Autonomy tests
  SimpleAutonomousTester.cpp
  Autonomous.cpp
  FakeMap.cpp
  mapping/ValidatorTest.cpp
  mapping/ObjectValidator.cpp
  mapping/EKFSlam/EKFSlam.cpp
  mapping/EKFSlam/tools.cpp)

add_executable(lidar_vis
    lidar/MainVis.cpp
    lidar/LidarVis.cpp
    lidar/PointCloudProcessing.cpp
    lidar/URGLidar.cpp)

add_executable(lidarOnlyTest
	mapping/EKFSlam/lidarOnlyTest.cpp
        mapping/EKFSlam/EKFSlam.cpp
        mapping/ObjectValidator.cpp
        mapping/EKFSlam/tools.cpp
        lidar/LidarVis.cpp
        lidar/PointCloudProcessing.cpp
        lidar/URGLidar.cpp)

add_executable(simulator_navigation_demo
  simulator/navigation.cpp
  simulator/plan.cpp
  simulator/search.cpp
  simulator/utils.cpp
  simulator/graphics.cpp
  simulator/simulator_world.cpp
  simulator/world.cpp)

target_link_libraries(lidar_vis liburg_c.a ${OpenCV_LIBS})
target_link_libraries(lidarOnlyTest liburg_c.a ${OpenCV_LIBS})
target_link_libraries(Rover ${OpenCV_LIBS})
target_link_libraries(simulator_navigation_demo sfml-graphics sfml-window sfml-system X11 pthread)

include(CTest)
include(Catch)
target_link_libraries(tests Catch2::Catch2)
target_link_libraries(tests ${OpenCV_LIBS})
catch_discover_tests(tests)

add_executable(Server
  Networking/Server.cpp)

add_executable(TCPClient
  Networking/TCPClient.cpp)

add_executable(FakeBaseStation
  Networking/FakeBaseStation.cpp)

add_executable(FakeCANBoard
  Networking/FakeCANBoard.cpp
  Networking/CANUtils.cpp
  Networking/TestPackets.cpp
  HindsightCAN/CANPacket.c
  HindsightCAN/CANCommon.c
  HindsightCAN/PortTemplate.c)

set_property(TARGET Rover PROPERTY CXX_STANDARD 14)
set_property(TARGET lidar_vis PROPERTY CXX_STANDARD 14)
set_property(TARGET lidarOnlyTest PROPERTY CXX_STANDARD 14)
