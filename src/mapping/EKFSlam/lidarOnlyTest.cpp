#include "EKFSlam.h"
#include "../../lidar/URGLidar.h"
#include "../../lidar/LidarVis.h"
#include "../../math/PointXY.h"


int main(int argc, char **argv)
{
	using namespace lidar;
	URGLidar lidar;
    EKFSlam ekf;
	if (!lidar.open())
	{
		std::cout << "failed to open lidar" << std::endl;
		return lidar.getError();
	}

	LidarVis vis(vis_win_width, vis_win_height, vis_bg_color, lidar_max_range);
	cv::namedWindow(vis_win_name);
	while (true)
	{
		vis.clear();
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

		vis.drawPoints(pts, vis_pt_color, vis_pt_radius);
		std::vector<std::vector<PointXY>> clusters = clusterPoints(pts, cluster_sep_threshold);
		ekf.updateFromLidar(clusters);
        for (std::vector<::PointXY> cluster : clusters)
		{
			std::vector<::PointXY> bounds = convexHull(cluster);
			vis.outlinePolygon(bounds, vis_conv_hull_color);
		}
        std::vector<ObstaclePoint> obstacles = ekf.getObstacles();
        std::vector<PointXY> pointsToDraw;

        for(ObstaclePoint obs: obstacles) {
            PointXY point {obs.x, obs.y};
            pointsToDraw.push_back(point);
        }
        cv::Scalar obstacleColor(0, 255, 0);
        vis.drawPoints(pointsToDraw, obstacleColor, 4);
		cv::imshow(vis_win_name, vis.getView());
		if (cv::waitKey(5) == vis_win_esc)
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


