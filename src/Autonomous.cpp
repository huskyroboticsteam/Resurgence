#include "Autonomous.h"
#include "simulator/world_interface.h"
#include "simulator/utils.h"
#include <cmath>

constexpr float PI = M_PI;

Autonomous::Autonomous(PointXY _target) : target(_target)
{
    state = 0;
    targetHeading = -1;
    forwardCount = -1;
    rightTurn = false;
}

void Autonomous::autonomyIter() {
  transform_t gps = readGPS(); // <--- has some heading information
  transform_t odom = readOdom();
  points_t lidar = readLidarScan();
  points_t landmarks = readLandmarks();

  // TODO incredibly clever algorithms for state estimation
  // and path planning and control!
  //
  // Ben: better localization than just GPS
  //    EKF that outputs transforms into a map frame
  //  input:
  //    GPS reading, odom reading, old EKF estimate
  //  output:
  //    robot frame -> map frame transform
  //    GPS frame -> map frame transform
  //
  // Jonah: map compositing
  //    taking data from different points in time and combining
  //    into a single map
  //  input:
  //    current lidar scan, past composite list of lidar scans
  //    robot frame -> map frame transform
  //  output:
  //    map (provided as list of points)
  //    list of locations in map frame for the AR tags we've seen so far
  //
  // Atharva: "mission planning" -- higher level planning
  //    driving through gates
  //    deciding when we're close enough to posts
  //  input: internal state, current URCLeg information
  //    currently visible AR tags in robot frame
  //    robot frame -> map frame transform
  //    GPS frame -> map frame transform
  //  output: goal pose in map frame
  //
  // Assaf: taking a map and computing a plan
  //  input: pose of robot in map frame, map (provided as a list of points), goal pose in map frame
  //  output: movement command
  //

  setCmdVel(0.5, 1.0);
}

void Autonomous::setWorldData(std::shared_ptr<WorldData> wdata)
{
    this->worldData = wdata;
}

std::pair<float, float> Autonomous::getDirections(float currHeading)
{
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
                          std::pair<float, float> directions)
{
    float speed = worldData->targetDistance();
    if (!worldData->lidarSees() || speed != 1)
    { // no obstacles in front
        if (forwardCount > 0 || forwardCount < 0)
        { //move forward
            directions = std::make_pair(currHeading, speed);
            state = 1;
            forwardCount--;
            return directions;
        }
        else
        { //moved forwards for a set number of times, now turn
            forwardCount = -1;
            rightTurn = false; //moved forward enough times without obstruction, next turn to target
            return stateTurn(currHeading, directions);
        }
    }
    else
    { // lidar now sees something
        return stateBackwards(currHeading, directions);
    }
}

std::pair<float, float>
Autonomous::stateTurn(float currHeading, std::pair<float, float> directions)
{
    if ((state == 0) && (currHeading == targetHeading))
    { // done turning
        return stateForwards(currHeading, directions);
    }
    else
    { // find heading to turn to
        if (rightTurn || forwardCount > 0)
        { // turn right if last turn was towards target or was obstructed when moving forwards
            targetHeading = currHeading + 45;
            forwardCount = 5; //reset forward count
        }
        else
        { //calculate angle to obstacle
            PointXY robotPos = worldData->getGPS();
            float x = target.x - robotPos.x;
            float y = target.y - robotPos.y;
            targetHeading = atan2f(y, x) * (180 / PI);
            targetHeading = 360 - targetHeading + 90;
            rightTurn = true; //next turn will turn right
        }
        if (targetHeading > 360)
        { //fixing angles over 360
            targetHeading = targetHeading - 360;
        }
        directions = std::make_pair(targetHeading, 0);
        state = 0;
        return directions;
    }
}

std::pair<float, float>
Autonomous::stateBackwards(float currHeading,
                           std::pair<float, float> directions)
{
    if (worldData->lidarSees())
    { // back up
        directions = std::make_pair(currHeading, -1);
        state = -1;
        return directions;
    }
    else
    { // done backing up, now turn
        return stateTurn(currHeading, directions);
    }
}

PointXY Autonomous::getTarget()
{
    return target;
}
