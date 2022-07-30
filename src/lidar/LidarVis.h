#pragma once

#include "PointCloudProcessing.h"
#include "PointGenerator.h"

#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace lidar {
constexpr int lidar_max_range = 10000;
constexpr int vis_win_width = 600;
constexpr int vis_win_height = 600;
constexpr int vis_pt_radius = 3;
constexpr int cluster_sep_threshold = 1000;
const std::string vis_win_name = "Lidar Visualizer";
constexpr char vis_win_esc = 'q';
const cv::Scalar vis_bg_color(255, 255, 255);
const cv::Scalar vis_pt_color(0, 0, 0);
const cv::Scalar vis_conv_hull_color(255, 0, 0);
const cv::Scalar vis_lidar_color(0, 0, 255);
const cv::Scalar vis_grid_color(0, 128, 0);
constexpr int vis_grid_dist = 1000;
constexpr int vis_lidar_size = 10;

class LidarVis {
private:
	cv::Mat view;
	cv::Scalar bg_color;
	int win_width;
	int win_height;
	int lidar_max_range;
	cv::Point worldToCvPoint(PointXY p);

public:
	LidarVis(int win_width, int win_height, cv::Scalar bgr, int max_range);
	void drawPoints(std::vector<PointXY>& pts, cv::Scalar bgr, int ptRadius);
	void outlinePolygon(std::vector<PointXY>& vertices, cv::Scalar bgr);
	void drawLidar(cv::Scalar bgr, int symb_size_px);
	void setGrid(cv::Scalar bgr, int scale);
	void clear();
	cv::Mat getView();
};
} // namespace lidar
