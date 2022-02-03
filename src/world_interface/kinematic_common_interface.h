#pragma once

#include "../kinematics/DiffDriveKinematics.h"

/**
 * @brief Call this method from all implementations of setCmdVel().
 * This is required, and will allow odometry methods to work correctly.
 * This also keeps track of the velocity to return for getCmdVel().
 *
 * @param wheelVels The tangential wheel velocities of the left and right wheels.
 */
void setCmdVelToIntegrate(const wheelvel_t& wheelVels);

/**
 * @brief Call this method from all implementations of setCmdVel().
 * This is required, and will allow odometry methods to work correctly.
 * This also keeps track of the velocity to return for getCmdVel().
 *
 * @param wheelVels The heading velocity (rad/s) and forward velocity (m/s).
 */
void setCmdVelToIntegrate(double dtheta, double dx);
