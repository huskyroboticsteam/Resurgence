cmake_minimum_required(VERSION 3.2)
set(CMAKE_CXX_STANDARD 11) # For the json library
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(Rover LANGUAGES CXX C)

set(USE_GPU OFF CACHE BOOL "Whether or not to compile GPU code")

if("${USE_GPU}")
  message(STATUS "GPU code is ON")
  add_definitions(-DWITH_GPU)
else()
  message(STATUS "GPU code is OFF")
endif()

set(AR_TESTS_WINDOWS OFF CACHE BOOL
	"Whether or not the AR unit tests should show images in windows")

if("${AR_TESTS_WINDOWS}")
  message(STATUS "AR unit tests: image windows are ON")
  add_definitions(-DAR_TESTS_WINDOWS)
else()
  message(STATUS "AR unit tests: image windows are OFF")
endif()

enable_testing()
find_package(Catch2 REQUIRED)

find_package(OpenCV CONFIG REQUIRED)
find_package(Threads REQUIRED)
add_subdirectory(ar)

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
  HindsightCAN/CANPacket.c
  HindsightCAN/CANCommon.c
  HindsightCAN/PortTemplate.c
  Autonomous.cpp
  Util.cpp
  simulator/utils.cpp
  )

list(APPEND RoverCommon
  Rover.cpp
  ${RoverCommonTest}
  Networking/Network.cpp
  mapping/EnvMap.cpp)

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
  simulator/noop_world.cpp
  # AR Detection tests
  ar/TagUnitTests.cpp
  ar/Tag.cpp
  # CAN tests
  Networking/tests.cpp
  Networking/TestPackets.cpp
  Networking/NetworkingStubs.cpp
  # Autonomy tests
  SimpleAutonomousTester.cpp
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

list(APPEND simulator_libs
  sfml-graphics sfml-window sfml-system X11 pthread)

target_link_libraries(lidar_vis liburg_c.a ${OpenCV_LIBS})
target_link_libraries(lidarOnlyTest liburg_c.a ${OpenCV_LIBS})
target_link_libraries(Rover ${OpenCV_LIBS})
target_link_libraries(RoverSim ${OpenCV_LIBS} ${simulator_libs})
target_link_libraries(simulator_navigation_demo ${simulator_libs})

include(CTest)
include(Catch)
target_link_libraries(tests Catch2::Catch2)
target_link_libraries(tests ${OpenCV_LIBS})
target_link_libraries(tests Threads::Threads)
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
