cmake_minimum_required(VERSION 3.2)

project(Rover LANGUAGES CXX)

#find_package(OpenCV CONFIG REQUIRED)
add_subdirectory(ar)

add_subdirectory(ObstacleSim)

add_executable(Rover
        Rover.cpp
        CommandLineOptions.cpp
        Globals.cpp
        Network.cpp
        mapping/EnvMap.cpp)

#target_link_libraries(Rover ${OpenCV_LIBS})

set_property(TARGET Rover PROPERTY CXX_STANDARD 14)
