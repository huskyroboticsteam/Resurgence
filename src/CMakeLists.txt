cmake_minimum_required(VERSION 3.2)

project(Rover LANGUAGES CXX)

set(USE_GPU OFF CACHE BOOL "Whether or not to compile GPU code")

if("${USE_GPU}")
  message(STATUS "GPU code is ON")
  add_definitions(-DWITH_GPU)
else()
  message(STATUS "GPU code is OFF")
endif()

enable_testing()
find_package(Catch2 REQUIRED)

find_package(OpenCV CONFIG REQUIRED)
find_package(Threads REQUIRED)
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
  ar/Tag.cpp)

target_link_libraries(Rover ${OpenCV_LIBS})
target_link_libraries(Rover Threads::Threads)

include(CTest)
include(Catch)
target_link_libraries(tests Catch2::Catch2)
target_link_libraries(tests ${OpenCV_LIBS})
target_link_libraries(tests Threads::Threads)
catch_discover_tests(tests)

set_property(TARGET Rover PROPERTY CXX_STANDARD 14)
