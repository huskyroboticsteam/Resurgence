cmake_minimum_required(VERSION 3.2)

project(Rover LANGUAGES CXX C)

#find_package(OpenCV CONFIG REQUIRED)

#TODO Possibly: Add a CMakeLists inside HindsightCAN submodule
add_executable(Rover
        Rover.cpp
        CommandLineOptions.cpp
        Globals.cpp
        Network.cpp
	HindsightCAN/CANPacket.c
        HindsightCAN/CANCommon.c)

#target_link_libraries(Rover ${OpenCV_LIBS})

set_property(TARGET Rover PROPERTY CXX_STANDARD 11)
