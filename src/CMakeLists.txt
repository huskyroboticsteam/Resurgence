cmake_minimum_required(VERSION 3.2)
cmake_policy(SET CMP0057 NEW)
set(CMAKE_CXX_STANDARD 11) # For the json library
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(Rover LANGUAGES CXX C)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
include_directories(${rclcpp_INCLUDE_DIRS})
include_directories(${geometry_msgs_INCLUDE_DIRS})

add_definitions(-DCHIP_TYPE=CHIP_TYPE_TEMPLATE)

enable_testing()

find_package(Catch2 REQUIRED)

add_library(hindsight_can SHARED
  HindsightCAN/CANPacket.c
  HindsightCAN/CANCommon.c
  HindsightCAN/PortTemplate.c
  )

add_library(simulator_base SHARED
  simulator/utils.cpp
  simulator/world.cpp
  simulator/graphics.cpp)

add_library(simulator_nav SHARED
  simulator/plan.cpp
  simulator/search.cpp
  )

add_library(rover_common SHARED
  CommandLineOptions.cpp
  Globals.cpp
  Networking/ParseBaseStation.cpp
  Networking/IK.cpp
  Networking/ParseCAN.cpp
  Networking/log.cpp
  Networking/motor_interface.cpp
  Util.cpp
  filters/PoseEstimator.cpp
  filters/PoseEstimatorLinear.cpp
  kinematics/DiffDriveKinematics.cpp
  worldmap/GlobalMap.cpp
  worldmap/TrICP.cpp
  )

add_library(real_base_station SHARED
  Networking/Network.cpp
  )

add_library(simulator_world_interface SHARED
  simulator/simulator_world.cpp
  )
add_library(stub_world_interface SHARED
  simulator/noop_world.cpp
  )

add_library(real_can_interface SHARED
  Networking/CANUtils.cpp
  )
add_library(stub_can_interface SHARED
  Networking/CANStubs.cpp
  )
list(APPEND rover_libs
  rover_common hindsight_can simulator_base simulator_nav)

add_executable(Rover
  Rover.cpp
  Autonomous.cpp
  )
target_link_libraries(Rover ${rover_libs}
  real_base_station
  real_can_interface
  stub_world_interface # only for now, until we figure out the real sensor interfaces
  )

add_executable(RoverNoCAN
  Rover.cpp
  Autonomous.cpp
  )
target_link_libraries(RoverNoCAN ${rover_libs}
  real_base_station
  stub_can_interface
  stub_world_interface
  )

add_executable(RoverSim
  Rover.cpp
  Autonomous.cpp
  )
target_link_libraries(RoverSim ${rover_libs}
  real_base_station
  stub_can_interface
  simulator_world_interface
  )

add_executable(PlanViz
  PlanViz.cpp)
target_link_libraries(PlanViz simulator_base)

add_executable(tests
  Tests.cpp
  Autonomous.cpp
  # AR Detection tests
  # ar/ARUnitTests.cpp
  ar/Tag.cpp
  # CAN tests
  Networking/tests.cpp
  Networking/TestPackets.cpp
  Networking/NetworkingStubs.cpp
  Networking/real_world_interface.cpp
  # Autonomy tests
  mapping/ValidatorTest.cpp
  ../tests/kinematics/DiffDriveKinematicsTest.cpp
  ../tests/filters/RollingAvgFilterTest.cpp
  ../tests/filters/EKFTest.cpp
  ../tests/filters/StateSpaceUtilsTest.cpp
  ../tests/worldmap/TrICPTest.cpp
  ../tests/worldmap/GlobalMapTest.cpp)
target_link_libraries(tests
  ${rover_libs}
  lidar_slam
  )

add_library(lidar_base
    lidar/LidarVis.cpp
    lidar/PointCloudProcessing.cpp
    lidar/URGLidar.cpp)

add_library(lidar_slam
        mapping/EKFSlam/EKFSlam.cpp
        mapping/ObjectValidator.cpp
        mapping/EKFSlam/tools.cpp
        )

add_executable(lidar_vis
    lidar/MainVis.cpp)
target_link_libraries(lidar_vis lidar_base)

add_executable(lidarOnlyTest
        mapping/EKFSlam/lidarOnlyTest.cpp
        )
target_link_libraries(lidarOnlyTest lidar_base lidar_slam)

add_executable(simulator_navigation_demo
  simulator/navigation.cpp
  )
target_link_libraries(simulator_navigation_demo
  simulator_base simulator_nav simulator_world_interface)

add_subdirectory(ar)

list(APPEND sfml_libs
  sfml-graphics sfml-window sfml-system pthread)

# My IDE kept yelling at me until I added these lines. I'm still able to build on the VM so I think it doesn't break anything
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

target_link_libraries(lidar_vis liburg_c.a ${OpenCV_LIBS})
target_link_libraries(lidarOnlyTest liburg_c.a ${OpenCV_LIBS})
target_link_libraries(Rover ${OpenCV_LIBS} ${rclcpp_LIBRARIES} ${geometry_msgs_LIBRARIES})
target_link_libraries(RoverNoCAN ${sfml_libs} ${rclcpp_LIBRARIES} ${geometry_msgs_LIBRARIES})
target_link_libraries(Rover ${sfml_libs})
target_link_libraries(RoverSim ${OpenCV_LIBS} ${sfml_libs} ${rclcpp_LIBRARIES} ${geometry_msgs_LIBRARIES})
target_link_libraries(simulator_navigation_demo ${sfml_libs})
target_link_libraries(tests ${sfml_libs} ${rclcpp_LIBRARIES} ${geometry_msgs_LIBRARIES})
target_link_libraries(PlanViz ${sfml_libs} ${rclcpp_LIBRARIES} ${geometry_msgs_LIBRARIES})

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
  Networking/TestPackets.cpp
  )
target_link_libraries(FakeCANBoard hindsight_can real_can_interface)

set_property(TARGET Rover PROPERTY CXX_STANDARD 14)
set_property(TARGET RoverNoCAN PROPERTY CXX_STANDARD 14)
set_property(TARGET RoverSim PROPERTY CXX_STANDARD 14)
set_property(TARGET PlanViz PROPERTY CXX_STANDARD 14)
set_property(TARGET tests PROPERTY CXX_STANDARD 14)
set_property(TARGET lidar_vis PROPERTY CXX_STANDARD 14)
set_property(TARGET lidarOnlyTest PROPERTY CXX_STANDARD 14)
