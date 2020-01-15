#include "LidarRead.h"

#include <iostream>


using namespace rp::standalone::rplidar;

void read_frame()
{
    RPlidarDriver *driver = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    _u32 baudrate = 256000;
    const char *com_path = "/dev/ttyUSB0";
    u_result rp_op_status;
    rplidar_response_device_info_t devinfo;
    if (!IS_OK(driver->connect(com_path, baudrate)))
    {
        std::cout << "failed to connect" << std::endl;
        exit(-1);
    }

    if (!IS_OK(driver->getDeviceInfo(devinfo)))
    {
        std::cout << "could not get device info" << std::endl;
        exit(-1);
    }

    if (!IS_OK(driver->startMotor()))
    {
        std::cout << "failed to start motor" << std::endl;
        exit(-1);
    }

    if (!IS_OK(driver->startScan(0, 1)))
    {
        std::cout << "failed to start scan" << std::endl;
        exit(-1);
    }

    rplidar_response_measurement_node_hq_t nodes[8192];
    size_t count = 8192;
    driver->grabScanDataHq(nodes, count);
    driver->ascendScanData(nodes, count);
    for (int i = 0; i < count; i++)
    {
        std::cout << "theta = " << nodes[i].angle_z_q14 * 90.0f / (1 << 14) << "\tdist = " << nodes[i].dist_mm_q2 << std::endl;
    }
}

int main(int argc, char **argv)
{
    read_frame();
    return 0;
}
