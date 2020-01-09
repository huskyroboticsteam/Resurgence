#pragma once

#include "PointCloudProcessing.h"
#include "PointGenerator.h"

#include <vector>
#include <set>

namespace Lidar
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
    void drawPoints(std::set<PointXY> &points, float r, float g, float b, float pt_size);
    void drawBoundingPolygon(std::vector<PointXY> &vertices, float r, float g,
                             float b, float line_width);
    void flushToDisplay();
};
} // namespace Lidar
