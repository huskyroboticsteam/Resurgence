#pragma once

#include "world_interface/data.h"

// This is a layer that provides further abstraction on top of
// the motor-level control provided by the world interface

namespace robot {

/**
 * @brief Request the robot to drive at the given velocities.
 *
 * @param dtheta The heading velocity.
 * @param dx The forward velocity.
 * @return double If the requested velocities are too high, they will be scaled down.
 * The returned value is the scale divisor. If no scaling was performed, 1 is returned.
 */
double setCmdVel(double dtheta, double dx);

/**
 * @brief Request the robot to drive in tank style, where each side is controlled individually.
 *
 * @param left The left velocity.
 * @param right The right velocity.
 * @return double If the requested velocities are too high, they will be scaled down.
 * The returned value is the scale divisor. If no scaling was performed, 1 is returned.
 */
double setTankCmdVel(double left, double right);

/**
 * @brief Set the power of the specified joint.
 *
 * @param joint The joint to set the power of.
 * @param power The power value to set, in the range [-1,1].
 */
void setJointPower(types::jointid_t joint, double power);

/**
 * @brief Set the position of the specified joint.
 *
 * @param joint the jointid_t of the joint to set the position of.
 * @param targetPos the position to set the joint to, in millidegrees.
 */
void setJointPos(types::jointid_t joint, int32_t targetPos);

/**
 * @param joint the jointid_t of the joint to get the position of.
 * @return the position of the joint specified by the jointid_t argument joint,
 * in millidegrees.
 */
types::DataPoint<int32_t> getJointPos(types::jointid_t joint);

void setScienceServoPos(uint8_t servoNum, int32_t angle);
} // namespace robot