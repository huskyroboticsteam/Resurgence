#include "Autonomous.h"
#include <cmath>

constexpr float PI = M_PI;

Autonomous::Autonomous(PointXY _target) :
    target(_target) {
    state = 0;
    targetHeading = -1;
    turnDirection = 0;
    forwardCount = -1;
    rightTurn = 1;
}

void Autonomous::setWorldData(std::shared_ptr<WorldData> wdata) {
    this->worldData = wdata;
}

std::pair<float, float> Autonomous::getDirections(float currHeading) {
    std::pair<float, float> directions; // heading, speed
    if (state == 1) 
    {
        return stateForwards(currHeading, directions);
    }
    if (state == 0) 
    {
        return stateTurn(currHeading, directions);
    }
    if (state == -1) 
    {
        return stateBackwards(currHeading, directions);
    }
}

std::pair<float, float>
Autonomous::stateForwards(float currHeading,
                          std::pair<float, float> directions) {
    float speed = worldData->targetDistance();
    if (!worldData->lidarSees() || speed != 1) 
    { // no obstacles in front
        if(forwardCount > 0 || forwardCount < 0) 
        { //move forward
            directions = std::make_pair(currHeading, speed);
            state = 1;
            forwardCount--;
            return directions;
        } else 
        { //moved forwards for a set number of times, now turn
            forwardCount = -1;
            return stateTurn(currHeading, directions);
        }
    } else 
    { // lidar now sees something
        return stateBackwards(currHeading, directions);
    }
}

std::pair<float, float>
Autonomous::stateTurn(float currHeading,std::pair<float, float> directions) {
    if ((state == 0) && (currHeading == targetHeading)) 
    { // done turning
        return stateForwards(currHeading, directions);
    } else 
    { // find heading to turn to
        if(turnDirection != 0) 
        {
            if (turnDirection == 1) 
            { // turn right
                targetHeading = currHeading + 90;
            }
            else if (turnDirection == -1) 
            { // turn left
                targetHeading = currHeading - 90;
            }
            turnDirection = 0;
            forwardCount = 5;
        } else 
        { //calculate angle to obstacle
            PointXY robotPos = worldData->getGPS();
            float x = target.x - robotPos.x;
            float y = target.y - robotPos.y;
            targetHeading = atan2f(y, x) * (180 / PI);
            targetHeading = 360 - targetHeading + 90;
            if (turnDirection == 1) 
            {
                turnDirection = -1;
            } else 
            {
                turnDirection = 1;
            }
        }
        if (targetHeading > 360) 
        {
            targetHeading = targetHeading - 360;
        }
        directions = std::make_pair(targetHeading, 0);
        state = 0;
        return directions;
    }
}

std::pair<float, float> 
Autonomous::stateBackwards(float currHeading,
                           std::pair<float, float> directions) {
    if (worldData->lidarSees()) 
    { // back up
        directions = std::make_pair(currHeading, -1);
        state = -1;
        return directions;
    } else 
    { // done backing up, now turn
        return stateTurn(currHeading, directions);
    }
}

PointXY Autonomous::getTarget() {
    return target;
}
