#include "PurePursuitCommand.h"
#include <algorithm>
#include <cmath>

using namespace navtypes;

namespace commands {


command_t PurePursuitCommand::getOutput() {

}   

point_t PurePursuitCommand::lineToCircleIntersection(point_t& p1, point_t& p2) {
    // refer to https://mathworld.wolfram.com/Circle-LineIntersection.html
    double currX = _pose(0);
    double currY = _pose(1);
    double x1 = p1[0] - _pose(0);
    double y1 = p1[1] - _pose(1);
    double x2 = p2[0] - _pose(0);
    double y2 = p2[1] - _pose(1);

    bool intersectionFound = false;

    double dx = x2 - x1;
    double dy = y2 - y1;
    double dr = sqrt(pow(dx, 2) + pow(dy, 2));

    double D = x1 * y2 - x2 * y1;

    // If discriminant is 0, there is 1 intersections
    //                 is > 0, there are 2 intersections
    //                 is < 0, there are no intersections
    double disc = pow(_lookahead_dist, 2) * pow(dr, 2) - pow(D, 2);

    if (disc >= 0) {
        double xSol1 = (D * dy + sign(dy) * dx * sqrt(disc)) / pow(dr, 2);
        double xSol2 = (D * dy - sign(dy) * dx * sqrt(disc)) / pow(dr, 2);
        double ySol1 = (-D * dx + abs(dy) * dx * sqrt(disc)) / pow(dr, 2);
        double ySol2 = (-D * dx - abs(dy) * dx * sqrt(disc)) / pow(dr, 2);

        point_t sol1 = {xSol1 + currX, ySol1 + currY};
        point_t sol2 = {xSol2 + currX, ySol2 + currY};

        double wMag1 = sqrt(pow((x1 - xSol1), 2) + pow((y1 - ySol1), 2));
        double wMag2 = sqrt(pow((x2 - xSol1), 2) + pow((y2 - ySol1), 2));

        // Let w be the vector from p1 to sol1. The proximity of sol1 to p1
        // can be found simply through the magnitude of w. We can scale the
        // proximity score with the distance from p1 to p2, such that the
        // score at p1 is 0 and the score at p2 is 1. The proximity score
        // of a viable solution must lie in [0, 1] to progress as a candidate.
        double proxScore1 = wMag1 / dr;
        double proxScore2 = wMag2 / dr;

        if ((0 <= proxScore1 && proxScore1 <= 1) || (0 <= proxScore2 && proxScore2 <= 1)) {
            if (proxScore2 < 0 || proxScore2 > 1) {
                return sol1;
            } else if (proxScore1 < 0 || proxScore1 > 1) {
                return sol2;
            } else {
                return proxScore2 > proxScore1 ? sol2 : sol1;
            }
        } else {
            // If no valid intersection is found, treat p2 as the intersection point.
            return p2;
        }
    }
}

double PurePursuitCommand::wrapAngle(double angErr) {
	return std::atan2(std::sin(angErr), std::cos(angErr));
}

// transform robo frame

// find curvature
double sign(double num) {
    if (num < 0) {
        return -1.0;
    } else {
        return 1.0;
    }
}

} // namespace commands