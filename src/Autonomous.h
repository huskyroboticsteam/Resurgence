#pragma once

#include <vector>
#include <cmath>
#include <memory>
#include "lidar/PointCloudProcessing.h"
#include "WorldData.h"
#include "FakeMap.h"
#include "Pathfinding/ObstacleMap.h"
#include "Pathfinding/Pather2.h"
#include "simulator/utils.h"

class Autonomous
{
public:
    Autonomous(PointXY);
    //Returns a pair of floats, in heading, speed
    //Accepts current heading of the robot as parameter
    std::pair<float, float> getDirections(float currHeading);
    //Gets the target's coordinate
    PointXY getTarget();
    void setWorldData(std::shared_ptr<WorldData>);
    void autonomyIter();

private:
    PointXY target;
    float targetHeading;
    int state;         //1 is move forwards, 0 is turning, -1 is back up
    bool rightTurn; //boolean for turning right or turning towards target
    int forwardCount; //Counter for how many times to move forwards after a set turn
    std::shared_ptr<WorldData> worldData;
    std::pair<float, float> stateForwards(float currHeading, std::pair<float, float> directions);
    std::pair<float, float> stateTurn(float currHeading, std::pair<float, float> directions);
    std::pair<float, float> stateBackwards(float currHeading, std::pair<float, float> directions);
    double pathDirection(points_t lidar, transform_t gps);

    //helpers to use simulator utils types
    // PointXY point_tToPointXY(point_t pnt);
    // std::vector<PointXY>& points_tToPointXYs(points_t pnts);
    
    ObstacleMap obsMap;
    Pather2 pather;
};
