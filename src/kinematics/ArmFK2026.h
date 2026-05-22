#pragma once

#include "../navtypes.h"
#include "ForwardArmKinematics.h"

#include <Eigen/Core>
#include <cmath>

namespace kinematics {

/**
 * @brief Kinematics object for the 2026 arm in 3D space.
 */
class ArmFK2026 : public ForwardArmKinematics<3, 3> {
public:
    /**
     * @brief Construct a new kinematics object.
     *
     * @param segLens The length of each arm segment (only shoulder and elbow).
     * @param jointMin The minimum angle of each joint.
     * @param jointMax The maximum angle of each joint.
     */
    ArmFK2026(const navtypes::Vectord<3>& segLens, 
               const navtypes::Vectord<3>& jointMin,
               const navtypes::Vectord<3>& jointMax)
        : segLens(segLens), jointMin(jointMin), jointMax(jointMax) {}

    navtypes::Vectord<3> getSegLens() const override {
        return segLens;
    }

    bool satisfiesConstraints(const navtypes::Vectord<3>& jointPos) const override {
        for (unsigned int i = 0; i < 3; i++) {
            if (jointPos[i] < jointMin[i] || jointPos[i] > jointMax[i]) {
                return false;
            }
        }
        return true;
    }

    navtypes::Vectord<3> jointPosToEEPos(const navtypes::Vectord<3>& jointPos) const override {
        double theta_B = jointPos[0];
        double theta_S = jointPos[1];
        double theta_E = jointPos[2];

        double l = segLens[1];
        double t = segLens[2];

        double x = std::cos(theta_B) * (l * std::sin(theta_S) + t * std::cos(theta_E));
        double y = std::sin(theta_B) * (l * std::sin(theta_S) + t * std::cos(theta_E));
        double z = l * std::cos(theta_S) - t * std::sin(theta_E);

        return Eigen::Vector3d(x, y, z);
    }

    navtypes::Matrixd<3, 3> getJacobian(const navtypes::Vectord<3>& jointPos) const override {
        double theta_B = jointPos[0];
        double theta_S = jointPos[1];
        double theta_E = jointPos[2];

        double l = segLens[1];
        double t = segLens[2];

        Eigen::Matrix<double, 3, 3> J;

        // Partial derivatives for X
        J(0, 0) = -std::sin(theta_B) * (l * std::sin(theta_S) + t * std::cos(theta_E));
        J(0, 1) =  std::cos(theta_B) * (l * std::cos(theta_S));
        J(0, 2) =  std::cos(theta_B) * (-t * std::sin(theta_E));

        // Partial derivatives for Y
        J(1, 0) =  std::cos(theta_B) * (l * std::sin(theta_S) + t * std::cos(theta_E));
        J(1, 1) =  std::sin(theta_B) * (l * std::cos(theta_S));
        J(1, 2) =  std::sin(theta_B) * (-t * std::sin(theta_E));

        // Partial derivatives for Z
        J(2, 0) =  0.0; // The base rotation does not affect the Z height
        J(2, 1) = -l * std::sin(theta_S);
        J(2, 2) = -t * std::cos(theta_E);

        return J;
    }

private:
    navtypes::Vectord<3> segLens;
    navtypes::Vectord<3> jointMin;
    navtypes::Vectord<3> jointMax;
};

} // namespace kinematics