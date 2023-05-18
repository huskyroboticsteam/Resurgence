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
	 * @param vel Velocity for end effector {x_velocity, y_velocity}.
	 * @param kin_obj PlanarArmKinematics object for the arm (should have the same number of
	 * arm joints).
	 */
	PlanarArmController(Eigen::Vector2d vel, PlanarArmKinematics<N> kin_obj)
		: velocity(vel), kinematics(kin_obj) {
		velTimestamp = robot::types::dataclock::now();
	}

	/**
	 * @brief Sets the x velocity for the end effector and returns the new command.
	 *
	 * @param currTime The current timestamp.
	 * @param targetVel The target x velocity.
	 * @param currJointPos The current joint positions.
	 * @return The new command, which is a tuple of [new joint positions, new joint
	 * velocities].
	 */
	std::tuple<navtypes::Vectord<N>, navtypes::Vectord<N>>
	set_x_vel(robot::types::datatime_t currTime, int32_t targetVel,
			  const navtypes::Vectord<N>& currJointPos) {
		velocity(0) = targetVel;
		velTimestamp = currTime;

		robot::types::datatime_t newTime = robot::types::dataclock::now();
		return getCommand(newTime, currJointPos);
	}

	/**
	 * @brief Sets the y velocity for the end effector and returns the new command.
	 *
	 * @param currTime The current timestamp.
	 * @param targetVel The target y velocity.
	 * @param currJointPos The current joint positions.
	 * @return The new command, which is a tuple of [new joint positions, new joint
	 * velocities].
	 */
	std::tuple<navtypes::Vectord<N>, navtypes::Vectord<N>>
	set_y_vel(robot::types::datatime_t currTime, int32_t targetVel,
			  const navtypes::Vectord<N>& currJointPos) {
		velocity(1) = targetVel;
		velTimestamp = currTime;

		robot::types::datatime_t newTime = robot::types::dataclock::now();
		return getCommand(newTime, currJointPos);
	}

	/**
	 * @brief Get the current command for the arm
	 *
	 * @param currTime The current timestamp.
	 * @param currJointPos The current joint positions.
	 * @return The new command, which is a tuple of [new joint positions, new joint
	 * velocities] to get to the new target end effector position.
	 */
	std::tuple<navtypes::Vectord<N>, navtypes::Vectord<N>>
	getCommand(robot::types::datatime_t currTime, const navtypes::Vectord<N>& currJointPos) {
		// get current EE position
		currEEPos = kinematics.jointPosToEEPos(currJointPos);

		// TODO: bound the current EE position

		// calculate target EE position
		double dt = util::durationToSec(currTime - velTimestamp);
		Eigen::Vector2d newPose = currEEPos + velocity * dt;

		// TODO: bound target EE position

		// get new joint positions for target EE
		navtypes::Vectord<N> joint_pos = kinematics.eePosToJointPos(newPose, currJointPos);

		// get new joint velocities for target EE
		navtypes::Vectord<N> joint_vel = kinematics.eeVelToJointVel(joint_pos, velocity);

		// return the command
		return std::make_tuple(joint_pos, joint_vel);
	}

private:
	robot::types::datatime_t velTimestamp;
	Eigen::Vector2d velocity{0.0, 0.0};
	PlanarArmKinematics<N> kinematics;
};
} // namespace kinematics