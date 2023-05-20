#pragma once

#include "../navtypes.h"
#include "PlanarArmKinematics.h"

#include <array>
#include <initializer_list>
#include <numeric>

#include <Eigen/Core>

namespace kinematics {

/**
 * @brief Controller to move planar arm to a target end effector position.
 *
 * @tparam N The number of arm joints.
 */
template <unsigned int N> class PlanarArmController {
public:
	/**
	 * @brief Construct a new controller object.
	 *
	 * @param currJointPos The current joint positions of the arm.
	 * @param kin_obj PlanarArmKinematics object for the arm (should have the same number of
	 * arm joints).
	 */
	PlanarArmController(const navtypes::Vectord<N>& currJointPos,
						PlanarArmKinematics<N> kin_obj)
		: kinematics(kin_obj) {
		setpoint = kinematics.jointPosToEEPos(currJointPos);
	}

	/**
	 * @brief Sets the end effector setpoint / target position.
	 *
	 * @param targetJointPos The target joint positions.
	 */
	void set_setpoint(const navtypes::Vectord<N>& targetJointPos) {
		setpoint = kinematics.jointPosToEEPos(targetJointPos);
	}

	/**
	 * @brief Gets the current end effector setpoint / target position.
	 *
	 * @param currTime The current timestamp.
	 * @return The target end effector position.
	 */
	Eigen::Vector2d get_setpoint(robot::types::datatime_t currTime) {
		// calculate current EE setpoint
		double dt = util::durationToSec(currTime - velTimestamp);
		return (setpoint + velocity * dt);
	}

	/**
	 * @brief Sets the x velocity for the end effector and returns the new command.
	 *
	 * @param currTime The current timestamp.
	 * @param targetVel The target x velocity.
	 * @return The new command, which is the new joint positions.
	 */
	void set_x_vel(robot::types::datatime_t currTime, double targetVel) {
		double dt = util::durationToSec(currTime - velTimestamp);
		setpoint += setpoint + velocity * dt;

		velocity(0) = targetVel;
		velTimestamp = currTime;
	}

	/**
	 * @brief Sets the y velocity for the end effector and returns the new command.
	 *
	 * @param currTime The current timestamp.
	 * @param targetVel The target y velocity.
	 * @return The new command, which is the new joint positions.
	 */
	void set_y_vel(robot::types::datatime_t currTime, double targetVel) {
		double dt = util::durationToSec(currTime - velTimestamp);
		setpoint += setpoint + velocity * dt;

		velocity(1) = targetVel;
		velTimestamp = currTime;
	}

	/**
	 * @brief Get the current command for the arm
	 *
	 * @param currTime The current timestamp.
	 * @param currJointPos The current joint positions.
	 * @return The new command, which is the new joint positions to get to the new target end
	 * effector position.
	 */
	navtypes::Vectord<N> getCommand(robot::types::datatime_t currTime,
									const navtypes::Vectord<N>& currJointPos) {
		Eigen::Vector2d newPos = get_setpoint(robot::types::datatime_t currTime);

		// bounds check (new pos + vel vector <= sum of joint lengths)
		double radius = kinematics.getSegLens().sum();
		if (newPos.norm() > radius) {
			// new position is outside of bounds
			// TODO: will need to eventually shrink velocity vector until it is within radius
			// instead of just normalizing it
			newPos.normalize();
		}
		setpoint = newPos;

		// get new joint positions for target EE
		return kinematics.eePosToJointPos(newPos, currJointPos);
	}

private:
	Eigen::Vector2d setpoint;
	Eigen::Vector2d velocity{0.0, 0.0};
	robot::types::datatime_t velTimestamp;
	PlanarArmKinematics<N> kinematics;
};
} // namespace kinematics