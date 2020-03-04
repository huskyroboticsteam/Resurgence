#include "LidarVis.h"

#include <iostream>

namespace lidar
{
LidarVis::LidarVis(int win_width, int win_height, std::vector<double> bgr, int max_range)
	: view(win_height, win_width, CV_8UC3, cv::Scalar(bgr[0], bgr[1], bgr[2])),
	  bg_color(cv::Scalar(bgr[0], bgr[1], bgr[2])), win_width(win_width),
	  win_height(win_height), lidar_max_range(max_range)
{
}

cv::Point LidarVis::worldToCvPoint(PointXY p)
{
	int x = (p.x / (2 * this->lidar_max_range) + 0.5) * this->win_width;
	int y = 0.5 * this->win_width * (1 - p.y / this->lidar_max_range);
	return cv::Point(x, y);
}

void LidarVis::drawPoints(std::vector<PointXY> &pts, std::vector<double> bgr, int pt_radius)
{
	this->view.setTo(this->bg_color);
	cv::Scalar pt_color(bgr[0], bgr[1], bgr[2]);
	for (PointXY p : pts)
	{
		cv::circle(this->view, worldToCvPoint(p), pt_radius, pt_color, -1);
	}
}

void LidarVis::outlinePolygon(std::vector<PointXY> &vertices, std::vector<double> bgr)
{
	cv::Scalar line_color(bgr[0], bgr[1], bgr[2]);
	for (int i = 0; i < vertices.size(); i++)
	{
		cv::Point p1 = worldToCvPoint(vertices[i]);
		cv::Point p2 = worldToCvPoint(vertices[(i + 1) % vertices.size()]);
		cv::line(this->view, p1, p2, line_color);
	}
}

cv::Mat LidarVis::getView()
{
	return this->view;
}
} // namespace lidar

int main(int argc, char **argv)
{
	using namespace lidar;
	URGLidar lidar;
	if (!lidar.open())
	{
		std::cout << "failed to open lidar" << std::endl;
		return lidar.getError();
	}

	float cluster_sep_thresh = 1000;

	LidarVis vis(600, 600, {255, 255, 255}, 5000);
	std::string win_name = "Lidar Visualization";
	cv::namedWindow(win_name);
	while (true)
	{
		if (!lidar.createFrame())
		{
			std::cout << "failed to create frame" << std::endl;
			return lidar.getError();
		}
		std::vector<Polar2D> polarPts = lidar.getLastFrame();
		std::vector<PointXY> pts(polarPts.size());
		for (Polar2D p : polarPts)
		{
			pts.push_back(lidar::polarToCartesian(p));
		}

		vis.drawPoints(pts, {0, 0, 0}, 3);
		std::vector<std::vector<PointXY>> clusters = clusterPoints(pts, cluster_sep_thresh);
		int cluster_count = 0;
		for (std::vector<PointXY> cluster : clusters)
		{
			cluster_count++;
			std::vector<PointXY> bounds = convexHull(cluster);
			vis.outlinePolygon(bounds, {200, 0, 0});
		}
		std::cout << "cluster count: " << cluster_count << std::endl;

		cv::imshow(win_name, vis.getView());
		if (cv::waitKey(5) == 'q')
		{
			break;
		}
	}
	if (!lidar.close())
	{
		std::cout << "failed to close lidar device" << std::endl;
		return lidar.getError();
	}
	return 0;
}
