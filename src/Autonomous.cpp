#include "Autonomous.h"
#include <cmath>

Autonomous::Autonomous(PointXY _target) :
    target(_target) {
    state = 1;
    targetHeading = -1;
    turnDirection = 0;
}

void Autonomous::setWorldData(std::shared_ptr<WorldData> wdata) {
    this->worldData = wdata;
}

// Function that returns heading and speed in a pair
// Accept robot's current heading as a parameter
std::pair<float, float> Autonomous::getDirections(float currHeading) {
    std::pair<float, float> directions; // heading, speed
    if (state == 1) {
        return stateForwards(currHeading, directions);
    }
    if (state == 0) {
        return stateTurn(currHeading, directions);
    }
    if (state == -1) {
        return stateBackwards(currHeading, directions);
    }
}

std::pair<float, float>
Autonomous::stateForwards(float currHeading,
                          std::pair<float, float> directions) {
    if (!worldData->lidarSees()) { // keep moving forwards
        directions = std::make_pair(currHeading, 1);
        state = 1;
        return directions;
    } else { // lidar now sees something
        return stateBackwards(currHeading, directions);
    }
}

std::pair<float, float>
Autonomous::stateTurn(float currHeading,
                      std::pair<float, float> directions) {
    if (currHeading == targetHeading) { // done turning
        return stateForwards(currHeading, directions);
    } else { // find heading to turn to
        if (state !=
            0) { // only doing calculations if just starting to turn
            if (turnDirection == 1) { // turn right
                targetHeading = 90;
                turnDirection = 0;
            } else if (turnDirection == -1) { // turn left
                targetHeading = 270;
                turnDirection = 0;
            } else {
                PointXY robotPos = worldData->getGPS();
                float x = target.x - robotPos.x;
                float y = target.y - robotPos.y;
                targetHeading = atan(y / x) * (180 / 3.14);
                targetHeading = 360 - targetHeading + 90;
                if (targetHeading > 360) {
                    targetHeading = targetHeading - 360;
                }
                if (turnDirection == -1) {
                    turnDirection = 1;
                } else {
                    turnDirection = -1;
                }
            }
        }
        directions = std::make_pair(targetHeading, 0);
        state = 0;
        return directions;
    }
}

std::pair<float, float>
Autonomous::stateBackwards(float currHeading,
                           std::pair<float, float> directions) {
    if (worldData->lidarSees()) { // back up
        directions = std::make_pair(currHeading, -1);
        state = -1;
        return directions;
    } else { // done backing up, now turn
        return stateTurn(currHeading, directions);
    }
}
