cmake_minimum_required(VERSION 3.2)

project(Rover LANGUAGES CXX)

#find_package(OpenCV CONFIG REQUIRED)
add_executable(Simulator
    Simulator.cpp)

# TODO link SFML
#target_link_libraries(Rover ${OpenCV_LIBS})

set_property(TARGET Simulator PROPERTY CXX_STANDARD 14)

