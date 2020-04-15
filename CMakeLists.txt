cmake_minimum_required(VERSION 3.15)
project(PY2020)

set(CMAKE_CXX_STANDARD 14)

include_directories(src)
include_directories(src/lidar)
include_directories(src/ObstacleSim)
include_directories(src/Pathfinding)

add_executable(PY2020)

