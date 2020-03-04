#pragma ONCE

#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "PointCloudProcessing.h"
#include "PointGenerator.h"
#include "URGLidar.h"

namespace lidar
{
class LidarVis
{
private:
	cv::Mat view;
	cv::Scalar bg_color;
	int win_width;
	int win_height;
	int lidar_max_range;

public:
	LidarVis(int win_width, int win_height, std::vector<double> bgr, int max_range);
	void drawPoints(std::vector<PointXY> &pts, std::vector<double> bgr, int ptRadius);
	void outlinePolygon(std::vector<PointXY> &vertices, std::vector<double> bgr);
	cv::Point worldToCvPoint(PointXY p);
	cv::Mat getView();
};
} // namespace lidar
