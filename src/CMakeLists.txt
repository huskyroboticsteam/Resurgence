cmake_minimum_required(VERSION 3.13) # Latest versions of Ubuntu should have at least 3.16;
                                     # argparse library requires 3.12.4 currently

set(CMAKE_CXX_STANDARD 17) # 11 For the json library, 17 for std library features
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(Rover LANGUAGES CXX C)

include(FetchContent)

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

##====== Module settings ======
# These variables control whether various modules are enabled or disabled. The goal is to allow
# a user to selectively exclude modules that they do not need.
set(GPS "ARDUPILOT" CACHE STRING "GPS interface to use.")
set(AVAILABLE_GPS "USB" "ARDUPILOT" "NONE")
set_property(CACHE GPS PROPERTY STRINGS ${AVAILABLE_GPS})
set(WITH_TESTS ON CACHE BOOL "Enable/Disable the tests.")
set(WORLD_INTERFACE "REAL" CACHE STRING "The world interface implementation to use.")
set(AVAILABLE_WORLD_INTERFACES "REAL" "SIMULATOR" "NO-OP")
set_property(CACHE WORLD_INTERFACE PROPERTY STRINGS ${AVAILABLE_WORLD_INTERFACES})

if(NOT WORLD_INTERFACE IN_LIST AVAILABLE_WORLD_INTERFACES)
  message(FATAL_ERROR "Invalid value for WORLD_INTERFACE: ${WORLD_INTERFACE}\n\
Valid values are ${AVAILABLE_WORLD_INTERFACES}")
endif()

if(NOT GPS IN_LIST AVAILABLE_GPS)
  message(FATAL_ERROR "Invalid value for GPS: ${GPS}\n\
Valid values are ${AVAILABLE_GPS}")
endif()

message("========================================\n\
Module settings:")
message("  GPS:\t\t\t${GPS}")
message("  Test Suite:\t\t${WITH_TESTS}")
message("  World Interface:\t${WORLD_INTERFACE}")
message("========================================")

##====== Dependencies ======
# Some of these are optional depending on module settings above.

# Get the Eigen linear algebra library. Eigen's CMake config includes its headers as user
# headers instead of system headers by default, so explicitly force them to be system includes
# here to avoid the compiler spamming warnings at us for stuff in the Eigen headers.
find_package(Eigen3 REQUIRED NO_MODULE)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories("/usr/local/include")

# Find the WebSocket++ library and Boost (provides the ASIO backend for Websocket++ and also
# provides the program_options argument parser). Only the `system` component of Boost is
# currently required.
find_package(websocketpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Find Frozen, library used for constexpr immutable containers
find_package(frozen REQUIRED)
# Frozen is just an interface library, only containing headers and some CMake settings. It
# should be fine to "link" the library to every target, since if the headers are not included,
# it will not have any effect on size or performance. This will help us not have to worry about
# making sure the headers are available in every file that includes Constants.h, for example.
link_libraries(frozen::frozen)

# Find argparse, library used for parsing command line arguments
find_package(argparse REQUIRED)


# Find the JSON library
find_package(nlohmann_json 3.2.0 REQUIRED)

# Find the CAN library; contains packet and protocol definitions and does not
# actually require physical CAN to be present.
FetchContent_Declare(
    HindsightCAN
    GIT_REPOSITORY https://github.com/huskyroboticsteam/HindsightCAN.git
    GIT_TAG 45b1544f20e7ba6dd0db19fb97e947430031fc6a
)
FetchContent_MakeAvailable(HindsightCAN)

FetchContent_Declare(LoguruGitRepo
	GIT_REPOSITORY "https://github.com/emilk/loguru"
	GIT_TAG        "master"
)
# set any loguru compile-time flags before calling MakeAvailable()
set(LOGURU_WITH_STREAMS TRUE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
FetchContent_MakeAvailable(LoguruGitRepo) # defines target 'loguru::loguru'
link_libraries(loguru::loguru -ldl)

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)

if(WITH_TESTS)
  find_package(Catch2 REQUIRED)
endif()

# Libraries needed to interface with real-life sensors. If we aren't using the real world
# interface, these aren't needed.
if(WORLD_INTERFACE STREQUAL "REAL")
  if(GPS STREQUAL "USB")
    # Find pkg-config (needed for libgps, as it doesn't have CMake configuration)
    find_package(PkgConfig REQUIRED)
    # Find the libgps USB GPS library.
    pkg_check_modules(libgps REQUIRED libgps)
  endif()
endif()

if(WITH_TESTS)
  enable_testing()
endif()

add_library(camera SHARED
  camera/Camera.cpp
)
target_link_libraries(camera PUBLIC ${OpenCV_LIBS})

# shared library for utility code
add_library(utils SHARED
  utils/core.cpp
  utils/random.cpp
  utils/ScopedTimer.cpp
  utils/transform.cpp
  utils/threading.cpp
  utils/math.cpp
  utils/json.cpp
  gps/gps_util.cpp
  base64/base64.cpp
  base64/base64_img.cpp
  kinematics/DiffDriveKinematics.cpp
  kinematics/DiffWristKinematics.cpp
  kinematics/SwerveDriveKinematics.cpp
  world_interface/data.cpp)
target_link_libraries(utils ${OpenCV_LIBS})

## ====== CAN Interfaces =======
# Stub CAN interface is used for the tests (and for the Rover if
# CAN is disabled) and the real CAN interface is used for the Rover if CAN is
# enabled.

# Common CAN source files

# **DON'T MAKE THIS SHARED**
# Making this library shared causes some memory fuckery
# No clue why but CAN I/O goes to shit. Don't do it.
add_library(can_interface STATIC)
target_sources(can_interface PRIVATE
  CAN/CANMotor.cpp
  CAN/CANUtils.cpp
)

# Hardware specific source files
target_sources(can_interface PRIVATE
  CAN/CAN.cpp)
target_link_libraries(can_interface PUBLIC
  HindsightCAN
  Threads::Threads
  utils
)

## GPS Implementations

add_library(gps_interface STATIC)
if(GPS STREQUAL "USB")
  target_sources(gps_interface PUBLIC gps/usb_gps/read_usb_gps.cpp)
  target_link_libraries(gps_interface ${libgps_LIBRARIES})
elseif(GPS STREQUAL "ARDUPILOT")
  target_sources(gps_interface PUBLIC
      ardupilot/ArduPilotProtocol.cpp
      ardupilot/ArduPilotInterface.cpp)
  target_link_libraries(gps_interface utils websocket_utils)
else()
  target_sources(gps_interface PUBLIC gps/dummy/dummy_gps.cpp)
endif()

## ====== World Interfaces =======
# hardware-agnostic utilities and common code for world interface
add_library(world_interface_core STATIC
  world_interface/gps_common_interface.cpp
  #ar/Detector.cpp
  #ar/MarkerSet.cpp
  #ar/MarkerPattern.cpp
  #ar/Tag.cpp
  #ar/read_landmarks.cpp
  world_interface/motor/base_motor.cpp
  )
target_include_directories(world_interface_core SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(world_interface_core PUBLIC ${vision_libs} opencv_aruco ${OpenCV_LIBS} utils camera)

# CAN library (above) requires some utilities from this
target_link_libraries(can_interface PUBLIC utils)

add_library(stub_world_interface STATIC
  world_interface/noop_world.cpp)

add_library(real_world_interface STATIC
  world_interface/real_world_interface.cpp
  world_interface/motor/can_motor.cpp
)

add_library(simulator_interface STATIC
  world_interface/simulator_interface.cpp
  world_interface/motor/sim_motor.cpp
)

target_link_libraries(real_world_interface gps_interface can_interface world_interface_core)
target_link_libraries(simulator_interface world_interface_core)
target_link_libraries(stub_world_interface world_interface_core)

if(WORLD_INTERFACE STREQUAL "REAL")
  add_library(world_interface ALIAS real_world_interface)
elseif(WORLD_INTERFACE STREQUAL "SIMULATOR")
  add_library(world_interface ALIAS simulator_interface)
else()
  add_library(world_interface ALIAS stub_world_interface)
endif()

add_library(filters SHARED
  filters/StateSpaceUtil.cpp
)
target_link_libraries(filters utils)

add_library(video STATIC
  video/H264Encoder.cpp
  video/encoder.cpp
)

target_link_libraries(video
  ${OpenCV_LIBS}
  x264 avutil avcodec avformat swscale
)

add_library(control SHARED
  control/TrapezoidalVelocityProfile.cpp
  control/SwerveController.cpp
)
target_link_libraries(control utils)

add_library(commands SHARED
  commands/DriveToWaypointCommand.cpp)

add_library(autonomous SHARED
  autonomous/AutonomousTask.cpp)
target_link_libraries(autonomous commands world_interface)

add_library(constants SHARED Constants.cpp)
link_libraries(constants)

add_executable(Rover Rover.cpp)
# **DON'T MAKE THIS SHARED**
# Making this library shared causes some memory fuckery
# No clue why but CAN I/O goes to shit. Don't do it.
add_library(rover_common STATIC
  Globals.cpp
  control_interface.cpp
  )
target_link_libraries(rover_common
  utils
  websocket_utils
  world_interface
)

list(APPEND rover_libs
  rover_common
  Threads::Threads
  websocket_utils
  filters
  control
  commands
  utils
)
target_link_libraries(Rover
  ${rover_libs}
  mission_control_interface
  world_interface
  ${vision_libs}
)

# add_executable(TunePID TunePID.cpp)
# target_link_libraries(TunePID
#   ${rover_libs}
#   can_interface
# )
# target_link_libraries(TunePID ${vision_libs})

if(WITH_TESTS)
  add_executable(tests
    Tests.cpp
    # AR Detection tests
    # ar/DetectorTests.cpp
    # ar/MarkerSetTests.cpp
    # Camera tests
    # ../tests/camera/CameraParamsTests.cpp
    # GPS tests
    ../tests/gps/GPSDatumTest.cpp
    ../tests/gps/GPSConverterTest.cpp
    # Controller tests
    ../tests/control/TrapezoidalVelocityProfileTest.cpp
    ../tests/control/JacobianControllerTest.cpp
    ../tests/control/PIDControllerTest.cpp
    ../tests/control/PlanarArmControllerTest.cpp
    # Kinematics tests
    ../tests/kinematics/DiffDriveKinematicsTest.cpp
    ../tests/kinematics/SwerveDriveKinematicsTest.cpp
    ../tests/kinematics/PlanarArmFKTest.cpp
    ../tests/kinematics/FabrikSolver2DTest.cpp
    # Filter tests
    ../tests/filters/RollingAvgFilterTest.cpp
    ../tests/filters/EKFTest.cpp
    ../tests/filters/MultiSensorEKFTest.cpp
    ../tests/filters/StateSpaceUtilsTest.cpp
    # Command tests
    ../tests/commands/DriveToWaypointCommandTest.cpp
    # Util tests
    ../tests/util/CoreTest.cpp
    ../tests/util/DataTest.cpp
    ../tests/util/MathTest.cpp
    ../tests/util/TimeTest.cpp
    ../tests/util/SchedulerTest.cpp
	# Protocol/teleop tests
	../tests/kinematics/DiffWristKinematicsTest.cpp)

  target_link_libraries(tests
    ${rover_libs}
    stub_world_interface
    ${OpenCV_LIBS})
  include(CTest)
  include(Catch)
  target_link_libraries(tests Catch2::Catch2)
  catch_discover_tests(tests)
endif()

# add_executable(gpsd_test
#   gps/gpsd_test.cpp)
# target_link_libraries(gpsd_test gps)

if (WORLD_INTERFACE STREQUAL "REAL")
  add_executable(LimitSwitchCalibration
    LimitCalib.cpp)
  target_link_libraries(LimitSwitchCalibration real_world_interface)
endif()

add_compile_options(
  -Wall
  -Wextra
  -Werror
)

add_subdirectory(ar)
add_subdirectory(camera)
add_subdirectory(CAN)
add_subdirectory(network)
