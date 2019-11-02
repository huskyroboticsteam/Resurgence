#include "Cluster.h"

#include <iostream>

std::shared_ptr<Point> polarTo2D(double r, double theta)
{   //returns a point struct containing an x,y cartesian coord relative to LIDAR reading
    Point p;
    p.xCoord = r * std::cos(theta);
    p.yCoord = r * std::sin(theta);
    return std::make_shared<Point>(p);
}

std::vector<std::shared_ptr<Point>> convertFrame(std::vector<std::shared_ptr<Polar2D>> frame) {
    std::vector<std::shared_ptr<Point>> cartesians;
    for(std::shared_ptr<Polar2D> p : frame) {
            cartesians.push_back(polarTo2D(p->r,p->theta));
    }
    return cartesians;
}

double calcDiff(std::shared_ptr<Point> p1, std::shared_ptr<Point> p2)
{ //Calculates difference between two points
    double xP = pow(p2->xCoord - p1->xCoord, 2);
    double yP = pow(p2->yCoord - p1->yCoord, 2);

    double diff = std::sqrt(xP + yP);
    return diff;
}


std::vector<std::set<std::shared_ptr<Point>>> filterPoints(
    std::vector<std::shared_ptr<Point>> points, float sep_distance)
{
    std::vector<std::shared_ptr<Object>> objects;
    Object lastObject;
    std::shared_ptr<Point> lastPoint = nullptr;
    for(int i = 1; i < points.size(); i++) {
        double diff = calcDiff(points[i], points[i-1]);
        if(diff < sep_distance) {
            if(lastPoint == points[i-1]) {
                lastObject.pts.push_back(points[i]);
            } else {
                Object obj;
                obj.pts.push_back(points[i-1]);
                obj.pts.push_back(points[i]);
                lastPoint = points[i];
                objects.push_back(std::make_shared<Object>(obj));
                lastObject = obj;
            }
        }
        if(lastPoint!=nullptr && calcDiff(points[i], lastPoint) < sizeOfRobot) {
            lastObject.pts.push_back(points[i]);
        }
    }
    //checks last point and first point
    double diff = calcDiff(points[points.size()-1], points[0]);
    if(diff < sep_distance) {
        if(lastPoint == points[points.size()-1]) {
            lastObject.pts.push_back(points[0]);
        } else {
            Object obj;
            obj.pts.push_back(points[points.size()-1]);
            obj.pts.push_back(points[0]);
            objects.push_back(std::make_shared<Object>(obj));
        }
    }
}