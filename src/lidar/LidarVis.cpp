#include "LidarVis.h"

#include <iostream>

namespace lidar
{
LidarVis::LidarVis(int win_width, int win_height, std::vector<double> rgb) : view(win_height, win_width, CV_8UC3, cv::Scalar(rgb[0], rgb[1], rgb[2])),
                                                                             bg_color(cv::Scalar(rgb[0], rgb[1], rgb[2])),
                                                                             win_width(win_width),
                                                                             win_height(win_height)
{
}

void LidarVis::drawPoints(std::vector<PointXY> &pts, std::vector<double> rgb, int ptRadius,
                          int max_range)
{
    this->view.setTo(this->bg_color);
    cv::Scalar ptColor(rgb[0], rgb[1], rgb[2]);
    for (PointXY pt : pts)
    {
        int x = (pt.x / (2 * max_range) + 0.5) * this->win_width;
        int y = this->win_height - ((pt.y / (2 * max_range) + 0.5) * this->win_height);
        cv::circle(this->view, cv::Point(x, y), ptRadius, ptColor, -1);
    }
}

cv::Mat LidarVis::getView()
{
    return this->view;
}
} // namespace lidar

int main(int argc, char **argv)
{
    URGLidar lidar;
    if (!lidar.open())
    {
        std::cout << "failed to open lidar" << std::endl;
        return lidar.getError();
    }
    std::cout << "getting one frame from lidar as test read:" << std::endl;
    if (!lidar.createFrame())
    {
        std::cout << "failed to create test frame" << std::endl;
        return lidar.getError();
    }
    std::vector<Polar2D> testFrame = lidar.getLastFrame();
    for (Polar2D p : testFrame)
    {
        std::cout << "(" << p.r << ", " << p.theta << ") ";
    }
    std::cout << std::endl;

    lidar::LidarVis vis(600, 600, {255, 255, 255});
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
        std::vector<lidar::PointXY> pts;
        for (Polar2D p : polarPts)
        {
            pts.push_back(lidar::polarToCartesian(p));
        }

        // tmp points to test coord conversion
        std::vector<lidar::PointXY> tmppts;
        tmppts.push_back({-2000, 2000});
        tmppts.push_back({3000, 3000});
        tmppts.push_back({4000, 1500});

        vis.drawPoints(tmppts, {0, 0, 0}, 3, 5000);
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
