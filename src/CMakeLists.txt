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

list(APPEND RoverCommonTest
  CommandLineOptions.cpp
  Globals.cpp
  Networking/ParseBaseStation.cpp
  Networking/IK.cpp
  Networking/ParseCAN.cpp
  Networking/log.cpp
  Networking/motor_interface.cpp
  HindsightCAN/CANPacket.c
  HindsightCAN/CANCommon.c
  HindsightCAN/PortTemplate.c
  Pathfinding/ObstacleMap.cpp
  Pathfinding/Pather2.cpp
  Autonomous.cpp
  Util.cpp
  simulator/utils.cpp
  filters/PoseEstimator.cpp
  filters/PoseEstimator.h
  filters/RollingAvgFilter.h
  filters/KalmanFilter.h
  )

list(APPEND RoverCommon
  Rover.cpp
  ${RoverCommonTest}
  Networking/Network.cpp)

add_executable(Rover
  ${RoverCommon}
  simulator/noop_world.cpp
  Networking/CANUtils.cpp
  )

add_executable(RoverNoCAN
  ${RoverCommon}
  simulator/noop_world.cpp
  Networking/CANStubs.cpp
  )

add_executable(RoverSim
  ${RoverCommon}
  Networking/CANStubs.cpp
  #simulator/navigation.cpp
  #simulator/plan.cpp
  #simulator/search.cpp
  simulator/utils.cpp
  simulator/graphics.cpp
  simulator/simulator_world.cpp
  simulator/world.cpp)

add_executable(tests
  ${RoverCommonTest}
  Tests.cpp
  # AR Detection tests
  ar/TagUnitTests.cpp
  ar/Tag.cpp
  # CAN tests
  Networking/tests.cpp
  Networking/TestPackets.cpp
  Networking/NetworkingStubs.cpp
  Networking/real_world_interface.cpp
  # Autonomy tests
  SimpleAutonomousTester.cpp
  FakeMap.cpp
  mapping/ValidatorTest.cpp
  mapping/ObjectValidator.cpp
  mapping/EKFSlam/EKFSlam.cpp
  mapping/EKFSlam/tools.cpp
  # Pathfinding Tests
  ../tests/Pathfinding/ObstacleMapTest.cpp
  # ../tests/Pathfinding/PatherTest.cpp
  )

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
  simulator/world.cpp
  )

add_executable(visualizer_test
  Pathfinding/visualizer_test.cpp
  Pathfinding/Visualizer.cpp
  Pathfinding/Pather2.cpp
  Pathfinding/ObstacleMap.cpp
  )

list(APPEND simulator_libs
  sfml-graphics sfml-window sfml-system pthread)

# My IDE kept yelling at me until I added these lines. I'm still able to build on the VM so I think it doesn't break anything
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

target_link_libraries(lidar_vis liburg_c.a ${OpenCV_LIBS})
target_link_libraries(lidarOnlyTest liburg_c.a ${OpenCV_LIBS})
target_link_libraries(Rover ${OpenCV_LIBS})
target_link_libraries(visualizer_test ${OpenCV_LIBS})
target_link_libraries(RoverSim ${OpenCV_LIBS} ${simulator_libs})
target_link_libraries(simulator_navigation_demo ${simulator_libs})

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
