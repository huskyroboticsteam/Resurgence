#pragma once

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
	 * @param kin_obj PlanarArmKinematics object for the arm (should have the same number of
	 *                arm joints).
	 * @param safetyFactor the percentage factor to scale maximum arm extension radius by to
	 *                      prevent singularity lock. Must be in range (0, 1).
	 */
	PlanarArmController(kinematics::PlanarArmKinematics<N> kin_obj, const double safetyFactor)
		: kin(kin_obj), safetyFactor(safetyFactor) {
		CHECK_F(safetyFactor > 0.0 && safetyFactor < 1.0);

		for (unsigned int i = 0; i < kin_obj.getNumSegments(); i++) {
			CHECK_F(kin_obj.getSegLens()[i] > 0.0);
		}
	}

	/**
	 * @brief Constructs a copy of an existing PlanarArmController object.
	 *
	 * @param other The existing PlanarArmController to copy.
	 */
	PlanarArmController(PlanarArmController&& other)
		: kin(std::move(other.kin)), safetyFactor(other.safetyFactor) {
		std::lock_guard<std::mutex> lock(other.mutex);
		mutableFields = std::move(other.mutableFields);
	}

	/**
	 * @brief Instantiates the PlanarArmController with the current joint positions,
	 * 		  returning true if the joint positions are valid. If PlanarArmController is
	 * already initialized, the PlanarArmController is reinitialized with the supplied
	 * positions and this function returns true. Otherwise, controller gets uninitialized and
	 * function returns false.
	 *
	 * @param currJointPos The current joint positions of the arm.
	 * @return true iff the joint positions are within the robot's maximum arm extension
	 * 		   radius and the controller was successfully initialized in the function call.
	 */
	bool tryInitController(const navtypes::Vectord<N>& currJointPos) {
		std::lock_guard<std::mutex> lock(mutex);

		if (is_setpoint_valid(currJointPos, kin, safetyFactor)) {
			if (!mutableFields.has_value()) {
				mutableFields.emplace();
			}

			Eigen::Vector2d newSetPoint = kin.jointPosToEEPos(currJointPos);
			mutableFields->setpoint = normalizeEEWithinRadius(newSetPoint);
			return true;
		}

		mutableFields.reset();
		return false;
	}

	/**
	 * @brief Returns whether the target joint positions are within the arm controller's radius
	 * limit.
	 *
	 * @param targetJointPos The target joint positions.
	 * @return whether the target joint positions are within the arm controller's radius limit.
	 */
	static bool is_setpoint_valid(const navtypes::Vectord<N>& targetJointPos,
								  kinematics::PlanarArmKinematics<N> kin_obj,
								  double safetyFactor) {
		// Compute the new EE position to determine if it is within
		// safety factor * length of fully extended arm.
		double eeRadius = kin_obj.jointPosToEEPos(targetJointPos).norm();
		double maxRadius = kin_obj.getSegLens().sum() * safetyFactor;
		return eeRadius <= maxRadius;
	}

	/**
	 * @brief Sets the end effector setpoint / target position.
	 *
	 * @param targetJointPos The target joint positions.
	 */
	void set_setpoint(const navtypes::Vectord<N>& targetJointPos) {
		Eigen::Vector2d newSetPoint = kin.jointPosToEEPos(targetJointPos);
		std::lock_guard<std::mutex> lock(mutex);
		mutableFields->setpoint = normalizeEEWithinRadius(newSetPoint);
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
			pos = mutableFields->setpoint;
			if (mutableFields->velTimestamp.has_value()) {
				double dt =
					util::durationToSec(currTime - mutableFields->velTimestamp.value());
				pos += mutableFields->velocity * dt;
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
	 * @param jointPos The current joint angles of the arm.
	 * @return The new command, which is the new joint positions.
	 */
	void set_x_vel(robot::types::datatime_t currTime, double targetVel,
				   const navtypes::Vectord<N>& jointPos) {
		std::lock_guard<std::mutex> lock(mutex);
		if (mutableFields->velTimestamp.has_value()) {
			// If we recieve a request for 0 velocity and the y velocity is 0, the arm should
			// stop moving. Set its setpoint to the current joint position to ensure this.
			if (targetVel == 0.0 && mutableFields->velocity(1) == 0.0) {
				Eigen::Vector2d newSetPoint = kin.jointPosToEEPos(jointPos);
				mutableFields->setpoint = normalizeEEWithinRadius(newSetPoint);
			} else {
				double dt =
					util::durationToSec(currTime - mutableFields->velTimestamp.value());
				// bounds check (new pos + vel vector <= sum of joint lengths)
				mutableFields->setpoint = normalizeEEWithinRadius(
					mutableFields->setpoint + mutableFields->velocity * dt);
			}
		}

		mutableFields->velocity(0) = targetVel;
		mutableFields->velTimestamp = currTime;
	}

	/**
	 * @brief Sets the y velocity for the end effector and returns the new command.
	 *
	 * @param currTime The current timestamp.
	 * @param targetVel The target y velocity.
	 * @param jointPos The current joint angles of the arm.
	 * @return The new command, which is the new joint positions.
	 */
	void set_y_vel(robot::types::datatime_t currTime, double targetVel,
				   const navtypes::Vectord<N>& jointPos) {
		std::lock_guard<std::mutex> lock(mutex);
		if (mutableFields->velTimestamp.has_value()) {
			// If we recieve a request for 0 velocity and the x velocity is 0, the arm should
			// stop moving. Set its setpoint to the current joint position to ensure this.
			if (mutableFields->velocity(0) == 0.0 && targetVel == 0.0) {
				Eigen::Vector2d newSetPoint = kin.jointPosToEEPos(jointPos);
				mutableFields->setpoint = normalizeEEWithinRadius(newSetPoint);
			} else {
				double dt =
					util::durationToSec(currTime - mutableFields->velTimestamp.value());
				// bounds check (new pos + vel vector <= sum of joint lengths)
				mutableFields->setpoint = normalizeEEWithinRadius(
					mutableFields->setpoint + mutableFields->velocity * dt);
			}
		}

		mutableFields->velocity(1) = targetVel;
		mutableFields->velTimestamp = currTime;
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
		mutableFields->velTimestamp = currTime;
		if (!success) {
			LOG_F(WARNING, "IK Failure!");
			mutableFields->velocity.setZero();
		} else {
			mutableFields->setpoint = newPos;
		}
		return jp;
	}

private:
	struct MutableFields {
		Eigen::Vector2d setpoint;
		Eigen::Vector2d velocity;
		std::optional<robot::types::datatime_t> velTimestamp;
	};

	std::optional<MutableFields> mutableFields;
	std::mutex mutex;
	const kinematics::PlanarArmKinematics<N> kin;
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
			double diffDotProd =
				(eePos - mutableFields->setpoint).dot(mutableFields->setpoint);
			double differenceNorm = (eePos - mutableFields->setpoint).squaredNorm();
			double discriminant =
				std::pow(diffDotProd, 2) -
				differenceNorm * (mutableFields->setpoint.squaredNorm() - std::pow(radius, 2));
			double a = (-diffDotProd + std::sqrt(discriminant)) / differenceNorm;
			// new constrained eePos = (1 - a) * (ee inside circle) + a * (ee outside circle)
			eePos = (1 - a) * mutableFields->setpoint + a * eePos;
		}
		return eePos;
	}
};
} // namespace control