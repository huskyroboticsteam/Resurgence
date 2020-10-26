#pragma once

#include <Eigen/Core>

class PoseEstimator {
public:
    PoseEstimator(double trackWidth, const Eigen::Vector3d &stateStdDevs, const Eigen::Vector3d &measurementStdDevs, double dt);

    void update(double thetaVel, double xVel, const Eigen::Vector3d &gps);

    Eigen::Vector3d xHat;

private:
    double dt;
    double trackWidth;
    Eigen::Matrix3d stateCovariance;
    Eigen::Matrix3d measurementCovariance;
    Eigen::Matrix3d A, B, C, D;
    Eigen::Matrix3d gainMatrix;

    void predict(const Eigen::Vector3d &u);
    void correct(const Eigen::Vector3d &u, const Eigen::Vector3d &y);
    Eigen::Vector3d getPoseDiff(const Eigen::Vector3d &pose, double thetaVel, double xVel);
};
