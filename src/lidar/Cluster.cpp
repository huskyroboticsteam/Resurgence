#include "Cluster.h"

#include <cmath>

// returns a point struct containing an x,y cartesian coord relative to LIDAR reading
std::shared_ptr<PointXY> polarTo2D(float r, float theta)
{     
    PointXY p;
    p.x = r * cos(theta);
    p.y = r * sin(theta);
    return std::make_shared<PointXY>(p);
}

std::vector<std::shared_ptr<PointXY>> convertFrame(std::vector<std::shared_ptr<Polar2D>> frame)
{
    std::vector<std::shared_ptr<PointXY>> cartesians;
    for (std::shared_ptr<Polar2D> p : frame)
    {
        cartesians.push_back(polarTo2D(p->r, p->theta));
    }
    return cartesians;
}

// Calculates difference between two points
float distance(std::shared_ptr<PointXY> p1, std::shared_ptr<PointXY> p2)
{     
    float xP = powf(p2->x - p1->x, 2);
    float yP = powf(p2->y - p1->y, 2);

    float diff = sqrtf(xP + yP);
    return diff;
}

std::vector<std::set<std::shared_ptr<PointXY>>> filterPointXYs(
    std::vector<std::shared_ptr<PointXY>> points, float sep_threshold)
{
    std::vector<std::set<std::shared_ptr<PointXY>>> clusters;
    std::set<std::shared_ptr<PointXY>> lastCluster;
    std::shared_ptr<PointXY> lastPoint = nullptr;
    for (int i = 1; i < points.size(); i++)
    {
        float diff = distance(points[i], points[i - 1]);
        if (diff < sep_threshold)
        {
            if (lastPoint == points[i - 1])
            {
                lastCluster.insert(points[i]);
            }
            else
            {
                std::set<std::shared_ptr<PointXY>> cluster;
                cluster.insert(points[i - 1]);
                cluster.insert(points[i]);
                lastPoint = points[i];
                clusters.push_back(cluster);
                lastCluster = cluster;
            }
        }
        if (lastPoint != nullptr && distance(points[i], lastPoint) < sep_threshold)
        {
            lastCluster.insert(points[i]);
        }
    }
    //checks last point and first point
    float diff = distance(points[points.size() - 1], points[0]);
    if (diff < sep_threshold)
    {
        if (lastPoint == points[points.size() - 1])
        {
            lastCluster.insert(points[0]);
        }
        else
        {
            std::set<std::shared_ptr<PointXY>> cluster;
            cluster.insert(points[points.size() - 1]);
            cluster.insert(points[0]);
            clusters.push_back(cluster);
        }
    }
    return clusters;
}