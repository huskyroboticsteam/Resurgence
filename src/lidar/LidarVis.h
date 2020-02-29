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
    cv::Scalar bg_color;
    int win_width;
    int win_height;

public:
    LidarVis(int win_width, int win_height, std::vector<double> rgb);
    void drawPoints(std::vector<PointXY> &pts, std::vector<double> rgb, int ptRadius,
                              int max_range);
    cv::Mat getView();
};
} // namespace lidar
