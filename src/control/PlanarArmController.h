#pragma once

#include "../kinematics/PlanarArmKinematics.h"
#include "../navtypes.h"
#include "../world_interface/data.h"

#include <array>
#include <initializer_list>
#include <numeric>
#include <optional>

#include <Eigen/Core>

namespace control {

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
						kinematics::PlanarArmKinematics<N> kin_obj)
		: kin(kin_obj), setpoint(kin.jointPosToEEPos(currJointPos)), velocity({0.0, 0.0}) {}

	/**
	 * @brief Sets the end effector setpoint / target position.
	 *
	 * @param targetJointPos The target joint positions.
	 */
	void set_setpoint(const navtypes::Vectord<N>& targetJointPos) {
		setpoint = kin.jointPosToEEPos(targetJointPos);
	}

	/**
	 * @brief Gets the current end effector setpoint / target position.
	 *
	 * @param currTime The current timestamp.
	 * @return The target end effector position.
	 */
	Eigen::Vector2d get_setpoint(robot::types::datatime_t currTime) {
		// calculate current EE setpoint
		Eigen::Vector2d pos = setpoint;
		if (velTimestamp.has_value()) {
			double dt = util::durationToSec(currTime - velTimestamp.value());
			pos += velocity * dt;
		}

		// bounds check (new pos + vel vector <= sum of joint lengths)
		double radius = kin.getSegLens().sum();
		if (pos.norm() > radius) {
			// new position is outside of bounds
			// TODO: will need to eventually shrink velocity vector until it is within radius
			// instead of just normalizing it
			pos.normalize();
		}
		return pos;
	}

	/**
	 * @brief Sets the x velocity for the end effector and returns the new command.
	 *
	 * @param currTime The current timestamp.
	 * @param targetVel The target x velocity.
	 * @return The new command, which is the new joint positions.
	 */
	void set_x_vel(robot::types::datatime_t currTime, double targetVel) {
		if (velTimestamp.has_value()) {
			double dt = util::durationToSec(currTime - velTimestamp.value());
			setpoint += velocity * dt;
		}

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
		if (velTimestamp.has_value()) {
			double dt = util::durationToSec(currTime - velTimestamp.value());
			setpoint += velocity * dt;
		}

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
		Eigen::Vector2d newPos = get_setpoint(currTime);
		setpoint = newPos;

		// get new joint positions for target EE
		return kin.eePosToJointPos(newPos, currJointPos);
	}

private:
	kinematics::PlanarArmKinematics<N> kin;
	Eigen::Vector2d setpoint;
	Eigen::Vector2d velocity;
	std::optional<robot::types::datatime_t> velTimestamp;
};
} // namespace control