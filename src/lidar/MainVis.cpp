#include "LidarVis.h"
#include "URGLidar.h"
#include <RPLidar/rplidar.h>
#include "../Constants.h"

#include <iostream>
#include <chrono>
#include <fstream>
#include <thread>

enum Lidars {
	Hokuyo,
	RP,
	NONE
};

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
	RPLidar rp_lidar(Constants::Lidar::RP_PATH, baudrate);

	LidarVis vis(vis_win_width, vis_win_height, vis_bg_color, lidar_max_range);
	cv::namedWindow(vis_win_name);
	while (true) {
		vis.clear();
        if (auto scan = rp_lidar.poll()) {
			std::vector<PointXY> pts;
			double dtheta = (scan.value().angle_max-scan.value().angle_min)/(scan.value().ranges.size()-1);
			for (long unsigned i = 0; i < scan.value().ranges.size(); i++) {
				double rad = dtheta*i;
				double dist_mm = scan.value().ranges[i] * Constants::Lidar::MM_PER_M;

				Polar2D frame{dist_mm, rad};
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
		"Usage: lidar_vis <lidar_type> <baud_rate>\n"
		"        <lidar_type>   # \"hokuyo\" \"rp\"\n"
		"        <baud_rate>    # default=115200\n"
		);
}

Lidars hash(std::string lidar) {
	if (lidar == "hokuyo") return Hokuyo;
	if (lidar == "rp") return RP;
	return NONE;
}

/**
 * @brief Main Visualizer
 * Visualizes scanned data from inputted lidar by plotting objects
 * on a graph
 */
int main(int argc, char** argv) {
	unsigned long baudrate;
	int errorCode = 0;
	if (argc == 2) {
		baudrate = Constants::Lidar::RPLIDAR_A1_BAUDRATE;
	} else if (argc == 3) {
		baudrate = (unsigned long) atol(argv[2]);
	} else {
		help();
		return -1;
	}

	switch(hash(std::string(argv[1]))) {
		case Hokuyo:
			errorCode = runHokuyo();
			break;
		case RP:
			errorCode = runRPLidar(baudrate);
			break;
		case NONE:
			help();
			return -1;
	}
	
	return errorCode;
}
