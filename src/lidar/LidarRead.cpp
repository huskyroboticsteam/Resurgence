#include "LidarRead.h"
#include <iostream>

namespace lidar
{
RPLidar::RPLidar(_u32 baudrate, const char *com_path)
{
    using namespace std::chrono_literals;
    this->driver = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    rplidar_response_device_info_t dev_info;
    if (!IS_OK(this->driver->connect(com_path, baudrate)))
    {
        std::cout << "failed to connect" << std::endl;
        exit(-1);
    }
    if (!IS_OK(this->driver->getDeviceInfo(dev_info)))
    {
        std::cout << "failed to get device info" << std::endl;
        exit(-1);
    }
    if (!IS_OK(this->driver->startMotor()))
    {
        std::cout << "failed to start motor" << std::endl;
        exit(-1);
    }
    if (!IS_OK(this->driver->startScan(0, 1)))
    {
        std::cout << "failed to start scan" << std::endl;
        exit(-1);
    }
    std::cout << "initialized lidar successfully" << std::endl;
}

RPLidar::~RPLidar()
{
    delete this->driver;
}

std::vector<PointXY> RPLidar::scan_frame(size_t node_count)
{
    std::vector<PointXY> points(node_count);
    rplidar_response_measurement_node_hq_t nodes[node_count];
    driver->grabScanDataHq(nodes, node_count);
    driver->ascendScanData(nodes, node_count);
    for (rplidar_response_measurement_node_hq_t node : nodes)
    {
        PointXY p;
        p.x = node.dist_mm_q2 * (cosf(node.angle_z_q14 * M_PI / 2) / (1 << 14));
        p.y = node.dist_mm_q2 * (sinf(node.angle_z_q14 * M_PI / 2) / (1 << 14));
    }
    std::cout << "read frame of " << points.size() << std::endl;
    return points;
}
}
