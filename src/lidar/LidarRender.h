#pragma once

#include "PointCloudProcessing.h"

#include <vector>
#include <set>

namespace Lidar
{
class LidarRender
{
private:
public:
    LidarRender(int win_width, int win_height, std::string win_title);
    ~LidarRender();
    void setBackground(float r, float g, float b, float a);
    void drawPoints(std::set<std::shared_ptr<PointXY>>, float r, float g, float b);
    void drawBoundingPolygon(std::vector<std::pair<float, float>> vertices, float r, float g,
                             float b);
    void flushToDisplay();
};
} // namespace Lidar
