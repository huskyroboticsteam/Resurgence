#pragma ONCE

#include "PointCloudProcessing.h"
#include "PointGenerator.h"
#include "URGLidar.h"

#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace lidar
{
class LidarVis
{
private:
    cv::Mat view;

public:
    LidarVis(int win_width, int win_height, std::vector<double> rgb);
    void drawPoints(std::vector<PointXY> &pts, bool cluster, bool ptsOrdered,
                    int sep_threshold, std::vector<double> rgb, int ptRadius);
    void display();
};
} // namespace lidar
