find_package(OpenCV REQUIRED)
add_executable(ar_test ARTester.cpp)
target_link_libraries(ar_test ${OpenCV_LIBS})
