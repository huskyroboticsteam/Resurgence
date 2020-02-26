#include "LidarVis.h"

#include <iostream>

namespace lidar
{
LidarVis::LidarVis(int win_width, int win_height, std::vector<double> rgb) : view(win_height, win_width, CV_8UC3, cv::Scalar(rgb[0], rgb[1], rgb[2]))
{
}

void LidarVis::drawPoints(std::vector<PointXY> &pts, bool clusterOn, bool ptsOrdered,
                          int sep_threshold, std::vector<double> rgb, int ptRadius)
{
    cv::Scalar ptColor(rgb[0], rgb[1], rgb[2]);
    if (clusterOn)
    {
        std::vector<std::vector<PointXY>> clusters;
        if (ptsOrdered)
        {
            clusters = clusterOrderedPoints(pts, sep_threshold);
        }
        else
        {
            clusterPoints(pts, sep_threshold);
        }

        for (std::vector<PointXY> cluster : clusters)
        {
            for (PointXY pt : cluster)
            {
                cv::circle(this->view, cv::Point(pt.x, pt.y), ptRadius, ptColor);
            }

            std::vector<PointXY> hull = convexHull(cluster);
            for (int i = 0; i < hull.size() - 1; i++)
            {
                cv::line(this->view, cv::Point(hull[i].x, hull[i].y),
                         cv::Point(hull[i + 1].x, hull[i + 1].y), ptColor);
            }
            cv::line(this->view, cv::Point(hull[hull.size() - 1].x, hull[hull.size() - 1].y),
                     cv::Point(hull[0].x, hull[0].y), ptColor);
        }
    }
    else
    {
        for (PointXY pt : pts)
        {
            cv::circle(this->view, cv::Point(pt.x, pt.y), ptRadius, ptColor);
        }
    }
}

void LidarVis::display()
{
    cv::imshow("", this->view);
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
        std::cout << "(" << p.r << ", " << p.theta << ") " << std::endl;
    }
    std::cout << std::endl;
    lidar::LidarVis vis(600, 400, {1.0, 1.0, 1.0});
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
        vis.drawPoints(pts, true, true, 500, {0, 0, 0}, 3);
        vis.display();
    }
    return 0;
}
