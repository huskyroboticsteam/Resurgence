#include "FakeMap.h"
#include "Autonomous.h"

constexpr float PI = M_PI;

FakeMap::FakeMap(Autonomous &autonomous_) : autonomous(autonomous_)
{
    robotPos.x = 0;
    robotPos.y = 0;
    heading = 0;
    margin = pow(10, -5);
    steps = 0;
    target = autonomous.getTarget();
}

void FakeMap::addObstacle(PointXY first, PointXY second) {
    std::pair<PointXY, PointXY> obs = std::make_pair(first, second);
    obstacles.push_back(obs);
}

void FakeMap::callAutonomous()
{
    while (targetDistance() == 1) //while the target isn't nearby
    {
        std::pair<float, float> directions = autonomous.getDirections(getHeading());
        update(directions);
        std::cout << "x: " << robotPos.x << std::endl;
        std::cout << "y: " << robotPos.y << std::endl;
        std::cout << "heading: " << heading << std::endl;
        steps ++;
    } //target is nearby, move once more
    std::pair<float, float> directions = autonomous.getDirections(getHeading());
    update(directions);
    steps ++;
    std::cout << "x: " << robotPos.x << std::endl;
    std::cout << "y: " << robotPos.y << std::endl;
    std::cout << "heading: " << heading << std::endl;
    std::cout << "steps: " << steps << std::endl;
}

bool FakeMap::lidarSees()
{
    bool isObs = false;
    //find the point that is 1 unit in front of robot pos.
    float inFrontX = sin(heading * (PI / 180)) + robotPos.x;
    float inFrontY = cos(heading * (PI / 180)) + robotPos.y;
    std::vector<float> robotLine = makeLine(inFrontX, inFrontY, robotPos.x, robotPos.y);
    //Cycle through obstacles, find line made by obstacle points
    //Find the point of intersection between robot's trajectory and line made by obstacle
    for (int i = 0; i < obstacles.size(); i++)
    {
        std::vector<float> obsLine = makeLine(obstacles[i].second.x, obstacles[i].second.y,
                                              obstacles[i].first.x, obstacles[i].first.y);
        float determinant = (robotLine[0] * obsLine[1]) - (obsLine[0] * robotLine[1]);
        if (determinant == 0)
        {
            // Lines are parallel
            isObs = false;
        }
        else //there is a point of intersection
        {
            //find point of intersection
            float intersectX = ((robotLine[2] * obsLine[1]) - (obsLine[2] * robotLine[1])) / determinant;
            float intersectY = ((robotLine[0] * obsLine[2]) - (obsLine[0] * robotLine[2])) / determinant;
            //see if point of intersection lies between the obs and robot line segment
            bool xRobMin = std::min(robotPos.x, inFrontX) <= intersectX + margin;
            bool xRobMax = intersectX - margin <= std::max(robotPos.x, inFrontX);
            bool yRobMin = std::min(robotPos.y, inFrontY) <= intersectY + margin;
            bool yRobMax = intersectY - margin <= std::max(robotPos.y, inFrontY);
            bool r = (xRobMin && xRobMax && yRobMin && yRobMax);
            bool xObsMin = std::min(obstacles[i].first.x, obstacles[i].second.x) <= intersectX + margin;
            bool xObsMax = intersectX - margin <= std::max(obstacles[i].first.x, obstacles[i].second.x);
            bool yObsMin = std::min(obstacles[i].first.y, obstacles[i].second.y) <= intersectY + margin;
            bool yObsMax = intersectY - margin <= std::max(obstacles[i].first.y, obstacles[i].second.y);
            bool o = (xObsMin && xObsMax) && (yObsMin && yObsMax);
            isObs = o && r;
            if (isObs)
            {
                return isObs;
            }
        }
    }
    return isObs;
}

float FakeMap::targetDistance()
{
    //make line segment in front of robot
    float inFrontX = sin(heading * (PI / 180)) + robotPos.x;
    float inFrontY = cos(heading * (PI / 180)) + robotPos.y;
    std::vector<float> robotLine = makeLine(inFrontX, inFrontY, robotPos.x, robotPos.y);
    if ((abs((robotLine[0] * target.x) + (robotLine[1] * target.y)) - robotLine[2]) < 0.1)
    { //checking if target is directly in front of robot
        //check how far away the target is
        float distance = sqrt(pow(target.x - robotPos.x, 2.0) + pow(target.y - robotPos.y, 2.0));
        if (distance <= 1)
        {
            return distance;
        }
    }
    return 1; //returning 1 is target is not within 1 unit distance
}

PointXY FakeMap::getGPS()
{
    return robotPos;
}

float FakeMap::getHeading()
{
    return heading;
}

//updates the position and heading of the robot
void FakeMap::update(std::pair<float, float> directions)
{
    //update robotPos and heading based on the directions
    heading = directions.first;
    float x = sin(heading * (PI / 180)) * directions.second;
    float y = cos(heading * (PI / 180)) * directions.second;
    robotPos.x = robotPos.x + x;
    robotPos.y = robotPos.y + y;
}

std::vector<float> FakeMap::makeLine(float x2, float y2, float x1, float y1)
{
    float a = y2 - y1;
    float b = x1 - x2;
    float c = (a * x1) + (b * y1);
    std::vector<float> line{a, b, c};
    return line;
}
