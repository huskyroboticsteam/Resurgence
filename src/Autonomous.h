#pragma once
#include <vector>
#include "../src/lidar/Cluster.h"
#include "WorldData.h"
#include "../src/FakeMap.h"

class Autonomous {
    public:
        Autonomous(PointXY);
        //Returns a pair of floats, in heading, speed
        //Accepts parameter, current heading of the robot
        std::pair<float, float> getDirections (float currHeading);
        void setWorldData(std::shared_ptr<WorldData>);
    private:
        //State variable
        //1 is move forwards, 0 is turning, -1 is back up
        int state;
        float targetHeading;
        int turnDirection;
        std::shared_ptr<WorldData> worldData;
        std::pair<float, float> stateForwards (float currHeading, std::pair<float, float> directions);
        std::pair<float, float> stateTurn (float currHeading, std::pair<float, float> directions);
        std::pair<float, float> stateBackwards (float currHeading, std::pair<float, float> directions);
        PointXY target;
};