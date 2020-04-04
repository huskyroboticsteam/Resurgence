#pragma once

#include <vector>
#include <iostream>
#include <cmath>
#include "lidar/PointCloudProcessing.h"
#include "WorldData.h"

class Autonomous;

class FakeMap : public WorldData
{
public:
    FakeMap(Autonomous &autonomous_);
    //Checks if there is an obstacle 1 unit in front of the robot
    bool lidarSees() override;
    //Returns GPS of robot
    PointXY getGPS() override;
    //Returns heading of robot
    float getHeading() override;
    //starts the autonomous pathing
    void callAutonomous();
    //Returns the distance of the target if distance is <1
    float targetDistance();
    // Adds a line-shaped obstacle to the map with endpoints first and second
    void addObstacle(PointXY first, PointXY second);

private:
    //updates the robot's position and heading
    void update(std::pair<float, float> directions);
    //makes a line in the form of ax + by = c
    std::vector<float> makeLine(float x2, float y2, float x1, float y1);
    Autonomous &autonomous;
    float heading;
    PointXY robotPos;
    std::vector<std::pair<PointXY, PointXY>> obstacles;
    PointXY target;
    int steps;
    //Margin of error
    float margin;
};
