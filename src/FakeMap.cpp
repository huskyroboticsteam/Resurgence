#include "FakeMap.h"
#include "Autonomous.h"

FakeMap::FakeMap (Autonomous& autonomous_) : autonomous(autonomous_) {
    robotPos.x = 0;
    robotPos.y = 0;
    heading = 0;
    PointXY ob;
    ob.x = 0;
    ob.y = 2;
    PointXY ob1;
    ob1.x = 1;
    ob1.y = 2;
    std::pair<PointXY, PointXY> obs;
    obs = std::make_pair(ob, ob1);
    obstacles.push_back(obs);
}

void FakeMap::callAutonomous () {
    for(int i = 0; i < 3; i++) {
        std::pair<float, float> directions = autonomous.getDirections(getHeading());
        update(directions);
        std::cout << "x: " << robotPos.x << std::endl;
        std::cout << "y: " << robotPos.y << std::endl;
        std::cout << "heading: " << heading << std::endl;
    }
}

bool FakeMap::lidarSees() {
    //is there an obstacle within 1 unit in front of the robot's pos
    //find the point that is 1 unit in front of robot pos.
    float inFrontX = sin(heading * (3.14/180)) + robotPos.x;
    float inFrontY = cos(heading * (3.14/180)) + robotPos.y;
    std::vector<float> robotLine = makeLine(inFrontX, inFrontY, robotPos.x, robotPos.y);
    //Cycle through obstacles, find line made by obstacle points
    //Find the point of intersection between robot's trajectory and line made by obstacle
    for(int i = 0; i < obstacles.size(); i++) {
        std::vector<float> obsLine = makeLine(obstacles[i].first.x, obstacles[i].first.y, 
        obstacles[i].second.x, obstacles[i].second.y);
        float determinant = (robotLine[0] * obsLine[1]) - (obsLine[0] * robotLine[1]);
        if (determinant == 0)
        {
            // Lines are parallel
        }
        else  //there is a point of intersection
        {
            //find point of intersection
            float intersectX = ((robotLine[2] * obsLine[1]) - (obsLine[2] * robotLine[1]))/determinant;
            float intersectY = ((robotLine[0] * obsLine[2]) - (obsLine[0] * robotLine[2]))/determinant;
            //see if point of intersection lies between the two line segments
            if(std::min(robotPos.x, obstacles[i].first.x) <= intersectX <= std::max(robotPos.x, obstacles[i].first.x)
            && std::min(robotPos.y, obstacles[i].first.y) <= intersectY <= std::max(robotPos.y, obstacles[i].first.y)) {
                return true;
            }
        }
    }
    return false;
}

PointXY FakeMap::getGPS() {
    return robotPos;
}

float FakeMap::getHeading() {
    return heading;
}

//updates the position and heading of the robot
void FakeMap::update(std::pair<float, float> directions) {
    //update robotPos and heading based on the directions
    heading = directions.first;
    float x = sin(heading * (3.14/180)) * directions.second;
    float y = cos(heading * (3.14/180)) * directions.second;
    robotPos.x = robotPos.x + x;
    robotPos.y = robotPos.y + y;
}

//finds values for a line in form of ax + by = c
std::vector<float> FakeMap::makeLine(float x2, float y2, float x1, float y1) {
    float a = y2 - y1;
    float b = x1 - x2;
    float c = (a * x1) + (b * y2);
    std::vector<float> line{a, b, c}; 
    return line;
}

