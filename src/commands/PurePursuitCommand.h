#pragma once

#include "CommandBase.h"
#include "../navtypes.h"

namespace commands {
class PurePursuitCommand : CommandBase {
public:
    // 
    PurePursuitCommand();

    command_t getOutput();

    pose_t _pose;
    

private:
    /** 
     * @see https://mathworld.wolfram.com/Circle-LineIntersection.html
     */
    point_t lineToCircleIntersection(point_t& p1, point_t& p2);

    /**
     * Wrap angle error to [-pi, pi] to prevent 90+ degree turns
     */
    double wrapAngle(double angErr);
    double sign(double num);

    // transform robo frame

    // find curvature


    double _lookahead_dist;
    points_t _path;
    
    
};
} // namespace commands