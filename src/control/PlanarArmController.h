#pragma once

#include "../kinematics/PlanarArmKinematics.h"
#include "../navtypes.h"
#include "../world_interface/data.h"

#include <array>
#include <initializer_list>
#include <mutex>
#include <numeric>
#include <optional>

#include <Eigen/Core>

namespace control {

/**
 * @brief Controller to move planar arm to a target end effector position.
 *
 * This class is thread-safe.
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
	 * @param safetyFactor the percentage factor to scale maximum arm extension radius by to
	 * prevent singularity lock.
	 */
	PlanarArmController(const navtypes::Vectord<N>& currJointPos,
						kinematics::PlanarArmKinematics<N> kin_obj, const double safetyFactor)
		: kin(kin_obj), setpoint(kin.jointPosToEEPos(currJointPos)), velocity({0.0, 0.0}),
		  safetyFactor(safetyFactor) {
		assert(safetyFactor > 0.0 && safetyFactor < 1.0);
	}

	/**
	 * @brief Sets the end effector setpoint / target position.
	 *
	 * @param targetJointPos The target joint positions.
	 */
	void set_setpoint(const navtypes::Vectord<N>& targetJointPos) {
		Eigen::Vector2d newSetPoint = kin.jointPosToEEPos(targetJointPos);
		std::lock_guard<std::mutex> lock(mutex);
		double radius = kin.getSegLens().sum() * safetyFactor;
		setpoint = normalizeVectorWithinRadius(newSetPoint, radius);
	}

	/**
	 * @brief Normalize the input vector to have a set radius,
	 *  	  while maintaining the same direction it did before if it exceeds that set radius.
	 *
	 * @param input The input vector to normalize.
	 * @param radius The radius to normalize the vector to.
	 */
	Eigen::Vector2d normalizeVectorWithinRadius(Eigen::Vector2d input, double radius) {
		if (input.norm() > radius) {
			// TODO: will need to eventually shrink velocity vector until it is within radius
			// instead of just normalizing it.

			input.normalize();
			input *= radius;
		}

		return input;
	}

	/**
	 * @brief Gets the current end effector setpoint / target position.
	 *
	 * @param currTime The current timestamp.
	 * @return The target end effector position.
	 */
	Eigen::Vector2d get_setpoint(robot::types::datatime_t currTime) {
		Eigen::Vector2d pos;
		{
			std::lock_guard<std::mutex> lock(mutex);
			// calculate current EE setpoint
			pos = setpoint;
			if (velTimestamp.has_value()) {
				double dt = util::durationToSec(currTime - velTimestamp.value());
				pos += velocity * dt;
			}
		}

		// bounds check (new pos + vel vector <= sum of joint lengths)
		double radius = kin.getSegLens().sum() * safetyFactor;
		return normalizeVectorWithinRadius(pos, radius);
	}

	/**
	 * @brief Sets the x velocity for the end effector and returns the new command.
	 *
	 * @param currTime The current timestamp.
	 * @param targetVel The target x velocity.
	 * @return The new command, which is the new joint positions.
	 */
	void set_x_vel(robot::types::datatime_t currTime, double targetVel) {
		std::lock_guard<std::mutex> lock(mutex);
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
		std::lock_guard<std::mutex> lock(mutex);
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
		// lock after calling get_setpoint since that internally locks the mutex
		std::lock_guard<std::mutex> lock(mutex);
		setpoint = newPos;

		// get new joint positions for target EE
		return kin.eePosToJointPos(newPos, currJointPos);
	}

private:
	std::mutex mutex;
	const kinematics::PlanarArmKinematics<N> kin;
	Eigen::Vector2d setpoint;
	Eigen::Vector2d velocity;
	std::optional<robot::types::datatime_t> velTimestamp;
	const double safetyFactor;
};
} // namespace control