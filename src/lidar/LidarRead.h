#pragma once

#include "PointCloudProcessing.h"

#include <rplidar/rplidar.h>

namespace lidar
{
using namespace rp::standalone::rplidar;

class RPLidar
{
private:
    RPlidarDriver *driver;
    bool connect_lidar(_u32 baudrate, const char *com_path);
public:
    RPLidar(_u32 baudrate, const char *com_path);
    ~RPLidar();
    std::vector<PointXY> scan_frame(size_t node_count);
};
} // namespace lidar
