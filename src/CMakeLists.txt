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
	Util.cpp)

add_executable(tests
  Tests.cpp
  ar/TagUnitTests.cpp)

include(CTest)
include(Catch)
target_link_libraries(tests Catch2::Catch2)
target_link_libraries(tests ${OpenCV_LIBS})
catch_discover_tests(tests)

set_property(TARGET Rover PROPERTY CXX_STANDARD 14)
