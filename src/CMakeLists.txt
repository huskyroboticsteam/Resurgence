cmake_minimum_required(VERSION 3.2)

project(Rover LANGUAGES CXX)

#find_package(OpenCV CONFIG REQUIRED)
add_executable(Rover
        Rover.cpp
        CommandLineOptions.cpp
        Globals.cpp
        Network.cpp
        mapping/EnvMap.cpp)

add_executable(EnvMapTest
        mapping/EnvMapTest.cpp
        mapping/EnvMap.cpp)

#target_link_libraries(Rover ${OpenCV_LIBS})

set_property(TARGET Rover PROPERTY CXX_STANDARD 11)
set_property(TARGET EnvMapTest PROPERTY CXX_STANDARD 11)