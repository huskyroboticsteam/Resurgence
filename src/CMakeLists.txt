cmake_minimum_required(VERSION 3.8) # Latest versions of Ubuntu should have at least 3.8

set(CMAKE_CXX_STANDARD 17) # 11 For the json library, 17 for std library features
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(Rover LANGUAGES CXX C)

# If a build type was not explicitly specified, default to "RelWithDebInfo" (Same optimization
# as the "Release" configuration but with debugging symbols).
#
# To change, you can pass -DCMAKE_BUILD_TYPE=... where ... is the configuration you would like
# to use. For example, if you want to do a full debug build (little to no optimization, with
# debugging symbols), add -DCMAKE_BUILD_TYPE=Debug to your `cmake` command. You may want to do
# a full debug build if code you want to debug gets optimized out by the compiler.
set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "The type of build to perform.")
set(AVAILABLE_BUILD_TYPES "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${AVAILABLE_BUILD_TYPES})
message("========================================\n\
Building ${CMAKE_BUILD_TYPE} configuration...\n\
Add -DCMAKE_BUILD_TYPE=... to change\n\
========================================")

# Generate a compile_commands.json file by default unless explicitly disabled.
# This allows editors like Visual Studio Code to be able to index the project correctly and
# locate header files and libraries.
if(NOT CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE STRING "Enable/Disable output of compile commands
  during generation." FORCE)
endif()

# CAN should be disabled by default if we are not on a Linux system, since it depends on Linux
# kernel-specific header files.
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  set(WITH_CAN_DEFAULT ON)
else()
  set(WITH_CAN_DEFAULT OFF)
endif()

# GPS should be disabled on Windows (at least for now) since we haven't (yet) successfully
# gotten libgps to compile on Windows under MSYS2
if(WIN32)
  set(WITH_GPS_DEFAULT OFF)
else()
  set(WITH_GPS_DEFAULT ON)
endif()

##====== Module settings ======
# These variables control whether various modules are enabled or disabled. The goal is to allow
# a user to selectively exclude modules that they do not need; for example if we are just
# trying to drive the rover for a demo, we don't need computer vision or autonomous navigation
# code. We also need a way to disable CAN on non-Linux systems that don't support it, so this
# is a way to do that.
set(WITH_CV ON CACHE BOOL "Enable/Disable computer vision code.")
set(WITH_CAN ${WITH_CAN_DEFAULT} CACHE BOOL "Enable/Disable the CAN code. Disabled on non-Linux
systems.")
set(WITH_GPS ${WITH_GPS_DEFAULT} CACHE BOOL "Enable/Disable the USB GPS interface. Disabled on
Windows systems.")
set(WITH_LIDAR ON CACHE BOOL "Enable/Disable the USB LiDAR interface.")
set(WITH_TESTS ON CACHE BOOL "Enable/Disable the tests.")
set(WORLD_INTERFACE "REAL" CACHE STRING "The world interface implementation to use.")
set(AVAILABLE_WORLD_INTERFACES "REAL" "SIMULATOR" "NO-OP")
set_property(CACHE WORLD_INTERFACE PROPERTY STRINGS ${AVAILABLE_WORLD_INTERFACES})

if(NOT WORLD_INTERFACE IN_LIST AVAILABLE_WORLD_INTERFACES)
  message(FATAL_ERROR "Invalid value for WORLD_INTERFACE: ${WORLD_INTERFACE}\n\
Valid values are ${AVAILABLE_WORLD_INTERFACES}")
endif()

message("========================================\n\
Module settings:")
message("  Computer Vision:\t${WITH_CV}")
if(NOT WITH_CAN_DEFAULT)
  message("  CAN Networking:\t${WITH_CAN} (non-Linux system - disabled by default)")
else()
  message("  CAN Networking:\t${WITH_CAN}")
endif()
if(NOT WITH_GPS_DEFAULT)
  message("  USB GPS code:\t\t${WITH_GPS} (WINDOWS system - disabled by default)")
else()
  message("  USB GPS code:\t\t${WITH_GPS}")
endif()
message("  USB LiDAR code:\t${WITH_LIDAR}")
message("  Test Suite:\t\t${WITH_TESTS}")
message("  World Interface:\t${WORLD_INTERFACE}")
message("========================================")

##====== Dependencies ======
# Some of these are optional depending on module settings above. For example, the lidar
# libraries are not needed if using the simulator world interface.

# Get the Eigen linear algebra library. Eigen's CMake config includes its headers as user
# headers instead of system headers by default, so explicitly force them to be system includes
# here to avoid the compiler spamming warnings at us for stuff in the Eigen headers.
find_package(Eigen3 REQUIRED NO_MODULE)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

# Find the WebSocket++ library and Boost (provides the ASIO backend for Websocket++ and also
# provides the program_options argument parser). Only the `system` component of Boost is
# currently required.
find_package(websocketpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

find_package(Threads REQUIRED)
if(WITH_CV)
  find_package(OpenCV REQUIRED)
endif()
if(WITH_TESTS)
  find_package(Catch2 REQUIRED)
endif()

# Find the SFML graphics library, used for the planar simulator
find_package(SFML REQUIRED COMPONENTS graphics window system)
list(APPEND sfml_libs
  sfml-graphics sfml-window sfml-system Threads::Threads)

# Libraries needed to interface with real-life sensors. If we aren't using the real world
# interface, these aren't needed.
if(WORLD_INTERFACE STREQUAL "REAL")
  # Find pkg-config (needed for libgps, as it doesn't have CMake configuration)
  find_package(PkgConfig REQUIRED)
  # Find the libgps USB GPS library.
  # TODO: Remove this when interface with Electronics' IMU board is implemented.
  pkg_check_modules(libgps REQUIRED libgps)
  if(WITH_LIDAR)
	# Find the URGLidar library.
	# TODO: Decide if we want to use those RPLidars we found
	find_package(URGLidar REQUIRED)
  endif()
endif()

# Preprocessor flag required for the CAN library; see
# https://github.com/huskyroboticsteam/HindsightCAN#how-to-port-to-a-new-chip
add_definitions(-DCHIP_TYPE=CHIP_TYPE_TEMPLATE)

# Enable all warnings except a few (unused variable/parameter/result)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall \
-Wno-unused-variable \
-Wno-unused-but-set-variable \
-Wno-unused-result \
-Wno-unused-parameter") # -Wconversion") 

if(WITH_TESTS)
  enable_testing()
endif()

add_library(hindsight_can SHARED
  HindsightCAN/CANPacket.c
  HindsightCAN/CANCommon.c
  HindsightCAN/CANMotorUnit.c
  HindsightCAN/CANScience.c
  HindsightCAN/PortFiles/PortTemplate.c)

add_library(simulator_base SHARED
  simulator/utils.cpp
  simulator/world.cpp
  simulator/graphics.cpp)
target_link_libraries(simulator_base ${sfml_libs})

add_library(simulator_nav SHARED
  planning/plan.cpp)

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
  commands/DriveToGateNoCompass.cpp)
target_link_libraries(rover_common ${Boost_LIBRARIES} websocketpp::websocketpp)

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
  Networking/CANUtils.cpp)
list(APPEND rover_libs
  rover_common hindsight_can simulator_base simulator_nav hr_slam)

add_library(stub_can_interface SHARED
  log.cpp
  Networking/CANStubs.cpp)

list(APPEND real_world_libs
  real_world_interface
  ${libgps_LIBRARIES}
  lidar_base
  URGLidar::urg_c)

list(APPEND vision_libs
  ${OpenCV_LIBS}
  opencv_aruco)

# TODO: Clean all this up; ideally we should have one rover target with options like CAN/no
# CAN, world interface, etc chosen by CMake options

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
target_link_libraries(Rover ${vision_libs} ${sfml_libs})

add_executable(TunePID TunePID.cpp)
target_link_libraries(TunePID ${rover_libs}
  real_base_station
  real_can_interface
  ${real_world_libs}
  )
target_link_libraries(TunePID ${vision_libs} ${sfml_libs})

add_executable(PlanViz
  PlanViz.cpp)
target_link_libraries(PlanViz simulator_base ${sfml_libs})

if(WITH_TESTS)
  add_executable(tests
    Tests.cpp
    Autonomous.cpp
    rospub_test.cpp
    # AR Detection tests
    ar/DetectorTests.cpp
    ar/MarkerSetTests.cpp
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
  target_link_libraries(tests ${sfml_libs})
  target_link_libraries(tests Catch2::Catch2)
  target_link_libraries(tests ${OpenCV_LIBS} opencv_aruco)
  catch_discover_tests(tests)
endif()

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
add_subdirectory(Networking)
