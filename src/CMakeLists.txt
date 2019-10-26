cmake_minimum_required(VERSION 3.2)

project(Rover LANGUAGES CXX)

#find_package(OpenCV CONFIG REQUIRED)

add_subdirectory(lidar)
add_subdirectory(mapping)

add_executable(Rover
        Rover.cpp
        CommandLineOptions.cpp
        Globals.cpp
        Network.cpp)

#target_link_libraries(Rover ${OpenCV_LIBS})
