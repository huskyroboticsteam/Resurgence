#pragma once

#include "CommandBase.h"
#include "../navtypes.h"

namespace commands {
class PurePursuitCommand : CommandBase {
public:
    // 
    PurePursuitCommand(const navtypes::points_t& waypoints, double driveVel, double doneThresh);


    command_t getOutput();
    

private:
    /** 
     * @see c
     */
    point_t lineToCircleIntersection(navtypes::point_t& p1, navtypes::point_t& p2);

    void interpolatePoints(const points_t& waypoints);

    /**
     * Wrap angle error to [-pi, pi] to prevent 90+ degree turns
     */
    double wrapAngle(double angErr);

    /**
     * Helper function to determine "sign" of a number. Treats 0 as positive.
     * 
     * @returns 1 if num >= 0, -1 if num < 0
     * 
     * @see https://mathworld.wolfram.com/Circle-LineIntersection.html
     */
    double sgn(double num);

    // transform robo frame

    // find curvature


    pose_t _pose;
    double _drive_vel;
    double _done_thresh;
    double _lookahead_dist;
    bool _set_state_called_before_output;
    navtypes:: points_t _path;
    
    
};
} // namespace commands