#pragma once

#include "../Constants.h"
#include "../kinematics/PlanarArmKinematics.h"
#include "../navtypes.h"
#include "../utils/time.h"
#include "../world_interface/data.h"

#include <array>
#include <cmath>
#include <initializer_list>
#include <loguru.hpp>
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
template <unsigned int N>
class PlanarArmController {
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
		: kin(kin_obj), velocity({0.0, 0.0}), safetyFactor(safetyFactor) {
		// NOTE: currJointPos could extend beyond the safetyFactor, so safety factor
		//       normalization logic is performed.
		set_setpoint(currJointPos);
		CHECK_F(safetyFactor > 0.0 && safetyFactor < 1.0);
	}

	/**
	 * @brief Sets the end effector setpoint / target position.
	 *
	 * @param targetJointPos The target joint positions.
	 */
	void set_setpoint(const navtypes::Vectord<N>& targetJointPos) {
		Eigen::Vector2d newSetPoint = kin.jointPosToEEPos(targetJointPos);
		std::lock_guard<std::mutex> lock(mutex);
		setpoint = normalizeEEWithinRadius(newSetPoint);
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
		return normalizeEEWithinRadius(pos);
	}

	/**
	 * @brief Sets the x velocity for the end effector and returns the new command.
	 *
	 * @param currTime The current timestamp.
	 * @param targetVel The target x velocity.
	 * @return The new command, which is the new joint positions.
	 */
	void set_x_vel(robot::types::datatime_t currTime, double targetVel,
				   robot::types::DataPoint<navtypes::Vectord<Constants::arm::IK_MOTORS.size()>>
					   jointPos) {
		std::lock_guard<std::mutex> lock(mutex);
		if (velTimestamp.has_value()) {
			if (targetVel == 0.0 && velocity(1) == 0.0) {
				Eigen::Vector2d newSetPoint = kin.jointPosToEEPos(jointPos.getData());
				setpoint = normalizeEEWithinRadius(newSetPoint);
			} else {
				double dt = util::durationToSec(currTime - velTimestamp.value());
				// bounds check (new pos + vel vector <= sum of joint lengths)
				setpoint = normalizeEEWithinRadius(setpoint + velocity * dt);
			}
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
	void set_y_vel(robot::types::datatime_t currTime, double targetVel,
				   robot::types::DataPoint<navtypes::Vectord<Constants::arm::IK_MOTORS.size()>>
					   jointPos) {
		std::lock_guard<std::mutex> lock(mutex);
		if (velTimestamp.has_value()) {
			if (velocity(0) == 0.0 && targetVel == 0.0) {
				Eigen::Vector2d newSetPoint = kin.jointPosToEEPos(jointPos.getData());
				setpoint = normalizeEEWithinRadius(newSetPoint);
			} else {
				double dt = util::durationToSec(currTime - velTimestamp.value());
				// bounds check (new pos + vel vector <= sum of joint lengths)
				setpoint = normalizeEEWithinRadius(setpoint + velocity * dt);
			}
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

		// get new joint positions for target EE
		bool success = false;
		navtypes::Vectord<N> jp = kin.eePosToJointPos(newPos, currJointPos, success);
		velTimestamp = currTime;
		if (!success) {
			LOG_F(WARNING, "IK Failure!");
			velocity.setZero();
		} else {
			setpoint = newPos;
		}
		return jp;
	}

private:
	std::mutex mutex;
	const kinematics::PlanarArmKinematics<N> kin;
	Eigen::Vector2d setpoint;
	Eigen::Vector2d velocity;
	std::optional<robot::types::datatime_t> velTimestamp;
	const double safetyFactor;

	/**
	 * @brief Normalize the input vector (end-effector position) to have a set radius,
	 *  	  while maintaining the same direction it did before if it exceeds that set radius.
	 *
	 * @param eePos The end-effector position to normalize.
	 */
	Eigen::Vector2d normalizeEEWithinRadius(Eigen::Vector2d eePos) {
		double radius = kin.getSegLens().sum() * safetyFactor;
		if (eePos.norm() > (radius + 1e-4)) {
			// new position is outside of bounds. Set new EE setpoint so it will follow the
			// velocity vector instead of drifting along the radius.
			// setpoint = setpoint inside circle
			// eePos = new setpoint outside circle
			// radius = ||setpoint + a*(eePos - setpoint)||  solve for a
			double diffDotProd = (eePos - setpoint).dot(setpoint);
			double differenceNorm = (eePos - setpoint).squaredNorm();
			double discriminant =
				std::pow(diffDotProd, 2) -
				differenceNorm * (setpoint.squaredNorm() - std::pow(radius, 2));
			double a = (-diffDotProd + std::sqrt(discriminant)) / differenceNorm;
			// new constrained eePos = (1 - a) * (ee inside circle) + a * (ee outside circle)
			eePos = (1 - a) * setpoint + a * eePos;
		}
		return eePos;
	}
};
} // namespace control