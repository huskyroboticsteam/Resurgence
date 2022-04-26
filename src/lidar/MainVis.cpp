#include "LidarVis.h"

#include <iostream>
#include <chrono>
#include <fstream>
#include <thread>

/**
 * @brief Hokuyo Lidar
 * Runs Main Visualizer with a plugged in Hokuyo Lidar
 */
int runHokuyo() {
	using namespace lidar;
	URGLidar lidar;
	if (!lidar.open()) {
		std::cout << "failed to open hokuyo lidar" << std::endl;
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

/**
 * @brief RP Lidar
 * Runs Main Visualizer on any RP Lidar that uses the RP LIdar SDK
 */
int runRPLidar(unsigned long baudrate) {
	using namespace lidar;
	RPLidar rp_lidar("/dev/ttyUSB0", baudrate);

	LidarVis vis(vis_win_width, vis_win_height, vis_bg_color, lidar_max_range);
	cv::namedWindow(vis_win_name);
	while (true) {
		vis.clear();
        if (auto scan = rp_lidar.poll()) {
			std::vector<PointXY> pts;
			double dtheta = (scan.value().angle_max-scan.value().angle_min)/(scan.value().ranges.size()-1);
			for (long unsigned i = 0; i < scan.value().ranges.size(); i++) {
				double rad = dtheta*i;
				double MM_TO_M = 1000;
				double dist = scan.value().ranges[i] * MM_TO_M;

				Polar2D frame{dist, rad};
				pts.push_back(lidar::polarToCartesian(frame));
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

static void help() {
	printf("Lidar Visualizer. \n"
		"Usage: \n"
		"     -l=<lidar_num>         # 1=hokyuo, 2=rplidar"
		"     -b=<baudrate>          # set baudrate, defaults to 115200"
		);
}

/**
 * @brief Main Visualizer
 * Visualizes scanned data from inputted lidar by plotting objects
 * on a graph
 */
int main(int argc, char** argv) {
	unsigned long baudrate;
	int lidarType;
	int errorCode = 0;
	if (argc == 2) {
		baudrate = 112500;
		lidarType = atoi(argv[1]);
	} else if (argc == 3) {
		baudrate = (unsigned long) atol(argv[2]);
		lidarType = atoi(argv[1]);
	} else {
		help();
		return -1;
	}


	switch(lidarType) {
		case 1: errorCode = runHokuyo();
		case 2: errorCode = runRPLidar(baudrate);
	}
	
	return errorCode;
}
