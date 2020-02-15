#include "URGLidar.h"

URGLidar::URGLidar() : lastFrameTime(std::chrono::steady_clock::now())
{
}

bool URGLidar::open()
{
    int ret;
    ret = urg_open(&urg, URG_SERIAL, connect_device, connect_baudrate);
    if (ret != 0)
    {
        error = ret;
        return false;
    }
    //Tells the scanner to scan in its full range (-135 deg to 135 deg).
    int first_step = urg_deg2step(&urg, -135);
    int last_step = urg_deg2step(&urg, +135);
    int skip_step = 0;
    ret = urg_set_scanning_parameter(&urg, first_step, last_step, skip_step);
    if (ret != 0)
    {
        error = ret;
        return false;
    }
    error = 0;
    return true;
}

std::vector<Polar2D> URGLidar::getLastFrame()
{
    return lastFrame;
}

auto URGLidar::getLastFrameTime()
{
    return lastFrameTime;
}

bool URGLidar::createFrame()
{
    //How many scans to complete (1) and how many to skip (0).
    int ret = urg_start_measurement(&urg, URG_DISTANCE, 1, 0);
    if (ret != 0)
    {
        error = ret;
        return false;
    }
    //Allocate memory to store the length data the scan will generate.
    long *length_data = (long *)malloc(sizeof(long) * urg_max_data_size(&urg));
    lastFrameTime = std::chrono::steady_clock::now();
    //Record the time just before recording starts and start recording the frame.
    int length_data_size = urg_get_distance(&urg, length_data, NULL);

    //Build each shared Polar2D point and store them in a std::vector
    std::vector<Polar2D> frame;
    for (int i = 0; i < length_data_size; ++i)
    {
        Polar2D pd{length_data[i], urg_index2rad(&urg, i)};
        frame.push_back(pd);
    }
    lastFrame = frame;
    error = 0;
    return true;
}

int URGLidar::getError()
{
    return error;
}

bool URGLidar::close()
{
    urg_close(&urg);
    error = 0;
    return true;
}