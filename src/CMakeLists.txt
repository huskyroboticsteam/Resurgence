cmake_minimum_required(VERSION 3.2)

project(Rover LANGUAGES CXX)

find_package(OpenCV REQUIRED)
add_subdirectory(ar)
add_executable(Rover
        Rover.cpp
        CommandLineOptions.cpp
        Globals.cpp
        Network.cpp)

#target_link_libraries(Rover ${OpenCV_LIBS})
set_property(TARGET Rover PROPERTY CXX_STANDARD 11)
