cmake_minimum_required(VERSION 3.2)

cmake_policy(SET CMP0057 NEW)
set(CMAKE_CXX_STANDARD 17) # 11 For the json library, 14 for ROS2, 17 for std library features
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(Rover LANGUAGES CXX C)

find_package(OpenCV)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)
find_package(Catch2 REQUIRED)
find_package(URGLidar)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
include_directories(${rclcpp_INCLUDE_DIRS})
include_directories(${geometry_msgs_INCLUDE_DIRS})

add_definitions(-DCHIP_TYPE=CHIP_TYPE_TEMPLATE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall \
-Wno-unused-variable \
-Wno-unused-but-set-variable \
-Wno-unused-result \
-Wno-unused-parameter") # -Wconversion") 

enable_testing()

add_library(hindsight_can SHARED
  HindsightCAN/CANPacket.c
  HindsightCAN/CANCommon.c
  HindsightCAN/CANMotorUnit.c
  HindsightCAN/CANScience.c
  HindsightCAN/PortFiles/PortTemplate.c
  )

add_library(simulator_base SHARED
  simulator/utils.cpp
  simulator/world.cpp
  simulator/graphics.cpp)

add_library(simulator_nav SHARED
        planning/plan.cpp
  )

add_library(rover_common SHARED
  Rover.cpp
  Autonomous.cpp
  CommandLineOptions.cpp
  Globals.cpp
  log.cpp
  rospub.cpp
  Networking/ParseBaseStation.cpp
  Networking/IK.cpp
  Networking/ParseCAN.cpp
  Networking/motor_interface.cpp
  Util.cpp
  filters/PoseEstimator.cpp
  filters/PoseEstimatorLinear.cpp
  kinematics/DiffDriveKinematics.cpp
  worldmap/GlobalMap.cpp
  worldmap/TrICP.cpp
  worldmap/QuadTree.cpp
  commands/nogps/DriveToGate.cpp
  commands/nogps/DriveThroughGate.cpp
  commands/DriveToGateNoCompass.cpp
  )

add_library(real_base_station SHARED
  Networking/Network.cpp
  )

add_library(simulator_world_interface SHARED
  world_interface/simulator_world.cpp
  )
add_library(stub_world_interface SHARED
  world_interface/noop_world.cpp
  )
add_library(real_world_interface SHARED
  world_interface/real_world_interface.cpp
  lidar/read_hokuyo_lidar.cpp
  camera/Camera.cpp
  camera/CameraParams.cpp
  ar/Detector.cpp
  ar/MarkerSet.cpp
  ar/MarkerPattern.cpp
  ar/Tag.cpp
  ar/read_landmarks.cpp
  gps/read_usb_gps.cpp
  )
target_include_directories(real_world_interface SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(real_world_interface ${vision_libs})

add_library(hr_slam SHARED
  simulator/graph.cpp
  simulator/friendly_graph.cpp
  simulator/factors.cpp
  )
add_library(real_can_interface SHARED
  log.cpp
  Networking/CANUtils.cpp
  )
add_library(stub_can_interface SHARED
  Networking/CANStubs.cpp
  )
list(APPEND rover_libs
  rover_common hindsight_can simulator_base simulator_nav hr_slam)

list(APPEND sfml_libs
  sfml-graphics sfml-window sfml-system pthread)

list(APPEND real_world_libs
  real_world_interface
  gps
  lidar_base
  URGLidar::urg_c)

list(APPEND vision_libs
  ${OpenCV_LIBS}
  opencv_aruco)

add_executable(Rover rover_main.cpp)
target_link_libraries(Rover ${rover_libs}
  real_base_station
  real_can_interface
  ${real_world_libs}
  )
target_link_libraries(Rover ${vision_libs} ${rclcpp_LIBRARIES} ${geometry_msgs_LIBRARIES} ${sfml_libs})

add_executable(RoverNoCAN rover_main.cpp)
target_link_libraries(RoverNoCAN ${rover_libs}
  real_base_station
  stub_can_interface
  stub_world_interface
  )
target_link_libraries(RoverNoCAN ${sfml_libs} ${rclcpp_LIBRARIES} ${geometry_msgs_LIBRARIES})

add_executable(RoverSim rover_main.cpp)
target_link_libraries(RoverSim ${rover_libs}
  real_base_station
  stub_can_interface
  simulator_world_interface
  )
target_link_libraries(RoverSim ${sfml_libs} ${rclcpp_LIBRARIES} ${geometry_msgs_LIBRARIES})

add_executable(TunePID TunePID.cpp)
target_link_libraries(TunePID ${rover_libs}
  real_base_station
  real_can_interface
  ${real_world_libs}
  )
target_link_libraries(TunePID ${vision_libs} ${rclcpp_LIBRARIES} ${geometry_msgs_LIBRARIES} ${sfml_libs})

add_executable(PlanViz
  PlanViz.cpp)
target_link_libraries(PlanViz simulator_base)
target_link_libraries(PlanViz ${sfml_libs} ${rclcpp_LIBRARIES} ${geometry_msgs_LIBRARIES})

add_executable(tests
  Tests.cpp
  Autonomous.cpp
  # AR Detection tests
  ar/DetectorTests.cpp
  ar/MarkerSetTests.cpp
  # CAN tests
  Networking/tests.cpp
  Networking/TestPackets.cpp
  Networking/NetworkingStubs.cpp
  # Camera tests
  camera/CameraParamsTests.cpp
  # Autonomy tests
  ../tests/kinematics/DiffDriveKinematicsTest.cpp
  ../tests/filters/RollingAvgFilterTest.cpp
  ../tests/filters/EKFTest.cpp
  ../tests/filters/StateSpaceUtilsTest.cpp
  ../tests/worldmap/TrICPTest.cpp
  ../tests/worldmap/GlobalMapTest.cpp
  ../tests/worldmap/QuadTreeTest.cpp)

target_link_libraries(tests
  ${rover_libs}
  stub_can_interface
  ${real_world_libs}
  )
include(CTest)
include(Catch)
target_link_libraries(tests ${sfml_libs} ${rclcpp_LIBRARIES} ${geometry_msgs_LIBRARIES})
target_link_libraries(tests Catch2::Catch2)
target_link_libraries(tests ${OpenCV_LIBS} opencv_aruco)
catch_discover_tests(tests)

add_library(lidar_base
    lidar/LidarVis.cpp
    lidar/PointCloudProcessing.cpp
    lidar/URGLidar.cpp)
target_include_directories(lidar_base SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})

add_executable(lidar_vis
    lidar/MainVis.cpp)
target_link_libraries(lidar_vis lidar_base)
target_link_libraries(lidar_vis liburg_c.a ${OpenCV_LIBS})

add_executable(gpsd_test
  gps/gpsd_test.cpp)
target_link_libraries(gpsd_test gps)

add_subdirectory(ar)
add_subdirectory(camera)

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

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
