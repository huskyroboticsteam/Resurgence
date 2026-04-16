#pragma once

#include "InverseArmKinematics.h"
#include <cmath>

namespace kinematics {

class 2026IKSolver : public InverseArmKinematics<3, 3> {
public:
    2026IKSolver(std::shared_ptr<const ForwardArmKinematics<3, 3>> fk) 
        : InverseArmKinematics<3, 3>(fk) {}

protected:
    navtypes::Vectord<3> solve(const navtypes::Vectord<3>& eePos,
                               const navtypes::Vectord<3>& currJointAngles,
                               bool& success) const override {
        
        navtypes::Vectord<3> targetAngles;

        double x = eePos[0];
        double y = eePos[1];
        double z = eePos[2];

        const double l = 0.5; // Shoulder-to-elbow length 
        const double t = 0.4; // Elbow-to-wrist length
        
        // Pre-calculate reused terms
        double r_xy = std::sqrt(x*x + y*y);
        double r_xyz = std::sqrt(x*x + y*y + z*z);

        // Calculate Base Angle
        double theta_B = std::atan2(y, x);

        // Calculate Shoulder Angle (with physical limit check)
        // The value inside the arccos function:
        double acos_inner = (l*l + x*x + y*y + z*z - t*t) / (2 * l * r_xyz);

        // Check if the target is physically reachable
        if (acos_inner < -1.0 || acos_inner > 1.0) {
            success = false; 
            break;
        }

        double theta_S = (M_PI / 2.0) - std::atan2(z, r_xy) - std::acos(acos_inner);

        // Calculate Elbow Angle
        double elbow_y = -(z - l * std::cos(theta_S));
        double elbow_x = r_xy - l * std::sin(theta_S);
        double theta_E = std::atan2(elbow_y, elbow_x);

        // Assign to the output vector
        targetAngles[0] = theta_B;
        targetAngles[1] = theta_S;
        targetAngles[2] = theta_E;

        // If made it this far, the calculation was physically possible
        success = true; 
        return targetAngles;
    }
};

} // namespace kinematics