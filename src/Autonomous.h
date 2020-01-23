#pragma once
#include <vector>
#include "../src/lidar/Cluster.h"

class Autonomous {
    public:
        Autonomous();
        //Returns a pair of floats, in heading, speed
        //Accepts parameter, current heading of the robot
        std::pair<float, float> getDirections (float currHeading);
    private:
        //State variable
        //1 is move forwards, 0 is turning, -1 is back up
        int state;
        float targetHeading;
        int turnDirection;
        std::pair<float, float> stateForwards (float currHeading, std::pair<float, float> directions);
        std::pair<float, float> stateTurn (float currHeading, std::pair<float, float> directions);
        std::pair<float, float> stateBackwards (float currHeading, std::pair<float, float> directions);

};