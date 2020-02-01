cmake_minimum_required(VERSION 3.2)

project(Rover LANGUAGES CXX C)

enable_testing()

find_package(Catch2 REQUIRED)

find_package(OpenCV REQUIRED)

add_definitions(-DCHIP_TYPE=CHIP_TYPE_TEMPLATE)

add_subdirectory(ar)

add_subdirectory(ObstacleSim)

add_executable(Rover
        Rover.cpp
        CommandLineOptions.cpp
        Globals.cpp
        Network.cpp
	Util.cpp
        mapping/EnvMap.cpp
        Networking/ParseCAN.cpp
        HindsightCAN/CANPacket.c
        HindsightCAN/CANCommon.c
        HindsightCAN/PortTemplate.c)

add_executable(tests
  Tests.cpp
  Util.cpp
  # AR Detection tests
  ar/TagUnitTests.cpp
  ar/Tag.cpp)

target_link_libraries(Rover ${OpenCV_LIBS})

include(CTest)
include(Catch)
target_link_libraries(tests Catch2::Catch2)
target_link_libraries(tests ${OpenCV_LIBS})
catch_discover_tests(tests)

set_property(TARGET Rover PROPERTY CXX_STANDARD 14)
