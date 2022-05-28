cmake_minimum_required(VERSION 3.8)

add_executable(FilterPerfTest FilterPerfTest.cpp)
target_link_libraries(FilterPerfTest ${rover_libs} simulator_interface)
