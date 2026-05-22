#pragma once

#include "InverseArmKinematics.h"
#include <cmath>

namespace kinematics {

class IKSolver2026 : public InverseArmKinematics<3, 3> {
public:
    IKSolver2026(std::shared_ptr<const ForwardArmKinematics<3, 3>> fk) 
        : InverseArmKinematics<3, 3>(fk) {}

protected:
    navtypes::Vectord<3> solve(const navtypes::Vectord<3>& eePos,
                               const navtypes::Vectord<3>& currJointAngles,
                               bool& success) const override {
        
        navtypes::Vectord<3> targetAngles;

        double x = eePos[0];
        double y = eePos[1];
        double z = eePos[2];

        navtypes::Vectord<3> segLens = this->fk->getSegLens();
        const double l = segLens[1]; 
        const double t = segLens[2]; 
        
        double r_xy = std::sqrt(x*x + y*y);
        double r_xyz = std::sqrt(x*x + y*y + z*z);

        double theta_B = std::atan2(y, x);

        double acos_inner = (l*l + x*x + y*y + z*z - t*t) / (2 * l * r_xyz);

        if (acos_inner < -1.0 || acos_inner > 1.0) {
            success = false; 
            return targetAngles; // Return early on failure
        }

        double theta_S = (M_PI / 2.0) - std::atan2(z, r_xy) - std::acos(acos_inner);

        double elbow_y = -(z - l * std::cos(theta_S));
        double elbow_x = r_xy - l * std::sin(theta_S);
        double theta_E = std::atan2(elbow_y, elbow_x);

        targetAngles[0] = theta_B;
        targetAngles[1] = theta_S;
        targetAngles[2] = theta_E;

        success = true; 
        return targetAngles;
    }
};

} // namespace kinematics