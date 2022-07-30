add_library(lidar_interface STATIC
  URGLidar.cpp
  PointCloudProcessing.cpp
  read_hokuyo_lidar.cpp)

find_package(RPLidar)

add_executable(lidar_vis
  MainVis.cpp
  LidarVis.cpp)
target_include_directories(lidar_vis SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(lidar_interface URGLidar::urg_c RPLidar::RPLidar utils)
target_link_libraries(lidar_vis lidar_interface ${OpenCV_LIBS})
