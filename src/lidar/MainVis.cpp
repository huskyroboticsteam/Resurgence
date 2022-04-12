#include "LidarVis.h"

#include <iostream>
#include <chrono>
#include <fstream>
#include <thread>

int runHokuyo() {
	using namespace lidar;
	URGLidar lidar;
	if (!lidar.open()) {
		std::cout << "failed to open hokuyo idar" << std::endl;
		return lidar.getError();
	}

	LidarVis vis(vis_win_width, vis_win_height, vis_bg_color, lidar_max_range);
	cv::namedWindow(vis_win_name);
	while (true) {
		vis.clear();
		if (!lidar.createFrame()) {
			std::cout << "failed to create frame" << std::endl;
			return lidar.getError();
		}
		std::vector<Polar2D> polarPts = lidar.getLastFrame();
		std::vector<PointXY> pts(polarPts.size());
		for (Polar2D p : polarPts) {
			pts.push_back(lidar::polarToCartesian(p));
		}

		vis.setGrid(vis_grid_color, vis_grid_dist);
		vis.drawPoints(pts, vis_pt_color, vis_pt_radius);
		std::vector<std::vector<PointXY>> clusters = clusterPoints(pts, cluster_sep_threshold);
		for (std::vector<PointXY> cluster : clusters) {
			std::vector<PointXY> bounds = convexHull(cluster);
			vis.outlinePolygon(bounds, vis_conv_hull_color);
		}
		vis.drawLidar(vis_lidar_color, vis_lidar_size);

		cv::imshow(vis_win_name, vis.getView());
		if (cv::waitKey(5) == vis_win_esc) {
			break;
		}
	}

	if (!lidar.close()) {
		std::cout << "failed to close lidar device" << std::endl;
		return lidar.getError();
	}
	return 0;
}

int runRPLidar() {
	using namespace lidar;
	RPLidar rp_lidar("/dev/ttyUSB0");
	rp_lidar.setBaudrate(115200);

	LidarVis vis(vis_win_width, vis_win_height, vis_bg_color, lidar_max_range);
	cv::namedWindow(vis_win_name);
	while (true) {
		vis.clear();
        if (auto scan = rp_lidar.poll()) {
			// Converts data into Polar Coord
			std::vector<Polar2D> currFrames;
			double dtheta = (scan.value().angle_max-scan.value().angle_min)/(scan.value().ranges.size()-1);
			for (int i = 0; i < scan.value().ranges.size(); i++) {
				double rad = dtheta*i;
				double dist = scan.value().ranges[i] * 1000;

				Polar2D frame{dist, rad};
				currFrames.push_back(frame);
			}

			std::vector<PointXY> pts(currFrames.size());
			for (Polar2D polar : currFrames) {
				pts.push_back(lidar::polarToCartesian(polar));
			}

			vis.setGrid(vis_grid_color, vis_grid_dist);
			vis.drawPoints(pts, vis_pt_color, vis_pt_radius);
			std::vector<std::vector<PointXY>> clusters = clusterPoints(pts, cluster_sep_threshold);
			for (std::vector<PointXY> cluster : clusters) {
				std::vector<PointXY> bounds = convexHull(cluster);
				vis.outlinePolygon(bounds, vis_conv_hull_color);
			}
			vis.drawLidar(vis_lidar_color, vis_lidar_size);

			cv::imshow(vis_win_name, vis.getView());
			if (cv::waitKey(5) == vis_win_esc) {
				break;
			}
			
        } else {
			std::cout << "failed to get frame" << std::endl;
            return -1;
		}
	}
	return 0;
}

int main(int argc, char** argv) {
	if (argc != 2) {
		perror("not enough args");
		return -1;
	}
	
	int lidarType = atoi(argv[1]);
	int errorCode = 0;

	switch(lidarType) {
		case 1: errorCode = runHokuyo();
		case 2: errorCode = runRPLidar();
	}
	
	return 0;
}
