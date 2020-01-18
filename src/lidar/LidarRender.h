#pragma once

#include "PointCloudProcessing.h"
#include "LidarRead.h"

#include <vector>

namespace lidar
{
class LidarRender
{
private:
    float disp_x_range;
    float disp_y_range;

public:
    LidarRender(int win_width, int win_height, std::string win_title, float disp_limits_x_range,
                float disp_limits_y_range);
    ~LidarRender();
    void setBackground(float r, float g, float b, float a);
    void drawPoints(const std::vector<PointXY> &points, float r, float g, float b, float pt_size);
    void drawBoundingPolygon(std::vector<PointXY> &vertices, float r, float g,
                             float b, float line_width);
    void flushToDisplay();
};
} // namespace Lidar
