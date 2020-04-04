cmake_minimum_required(VERSION 3.2)

project(Rover LANGUAGES CXX)

enable_testing()

find_package(Catch2 REQUIRED)

find_package(OpenCV REQUIRED)

add_subdirectory(ar)

add_subdirectory(ObstacleSim)

add_executable(Rover
        Rover.cpp
        CommandLineOptions.cpp
        Globals.cpp
        Network.cpp
	      Util.cpp
        mapping/EnvMap.cpp)

add_executable(tests
  Tests.cpp
  Util.cpp
  # AR Detection tests
  ar/TagUnitTests.cpp
  ar/Tag.cpp
  SimpleAutonomousTester.cpp
  Autonomous.cpp
  FakeMap.cpp)

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

target_link_libraries(lidar_vis liburg_c.a ${OpenCV_LIBS})
target_link_libraries(lidarOnlyTest liburg_c.a ${OpenCV_LIBS})
target_link_libraries(Rover ${OpenCV_LIBS})

include(CTest)
include(Catch)
target_link_libraries(tests Catch2::Catch2 pthread)
target_link_libraries(tests ${OpenCV_LIBS})
catch_discover_tests(tests)

set_property(TARGET Rover PROPERTY CXX_STANDARD 14)
set_property(TARGET lidar_vis PROPERTY CXX_STANDARD 14)
set_property(TARGET lidarOnlyTest PROPERTY CXX_STANDARD 14)
