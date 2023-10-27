#pragma once

#include "../navtypes.h"
#include "../world_interface/data.h"
#include "../utils/time.h"

#include <array>
#include <cmath>
#include <optional>
#include <vector>

#include <Eigen/Core>

namespace control {

/**
 * @brief Trapezoidal velocity profile in multiple dimensions.
 * Motion is coordinated so that all dimensions arrive at the goal at the same time.
 *
 * @tparam dim The number of dimensions that are controlled by this profile.
 */
template <int dim> class TrapezoidalVelocityProfile {
public:
	/**
	 * @brief Construct a new velocity profile object.
	 *
	 * @param maxVel The maximum velocity the profile should attain.
	 * @param maxAccel The maximum acceleration the profile should use.
	 */
	TrapezoidalVelocityProfile(double maxVel, double maxAccel)
		: maxVel(maxVel), maxAccel(maxAccel) {}

	/**
	 * @brief Reset the profile and clear any calculated profile.
	 */
	void reset() {
		profile.reset();
	}

	/**
	 * @brief Check if a profile has been calculated.
	 *
	 * @return bool True iff setTarget() has been called since construction or the last reset.
	 */
	bool hasTarget() const {
		return profile.has_value();
	}

	/**
	 * @brief Calculate a profile, assuming the mechanism begins and ends at rest.
	 *
	 * @param currTime The current timestamp.
	 * @param startPos The position the profile starts at.
	 * @param endPos The position the profile ends at.
	 */
	void setTarget(robot::types::datatime_t currTime, const std::array<double, dim>& startPos,
				   const std::array<double, dim>& endPos) {
		navtypes::Vectord<dim> startVec, endVec;
		for (int i = 0; i < dim; i++) {
			startVec(i) = startPos[i];
			endVec(i) = endPos[i];
		}
		setTarget(currTime, startVec, endVec);
	}

	/**
	 * @brief Calculate a profile, assuming the mechanism begins and ends at rest.
	 *
	 * @param currTime The current timestamp.
	 * @param startPos The position the profile starts at.
	 * @param endPos The position the profile ends at.
	 */
	void setTarget(robot::types::datatime_t currTime, const navtypes::Vectord<dim>& startPos,
				   const navtypes::Vectord<dim>& endPos) {
		profile_t profile{.startTime = currTime, .startPos = startPos, .endPos = endPos};
		// min time required to accelerate to top speed
		double rampUpTime = maxVel / maxAccel;
		// min distance required to accelerate to top speed
		double rampUpDist = std::pow(maxVel, 2) / (2 * maxAccel);

		double dist = (endPos - startPos).norm();
		if (2 * rampUpDist < dist) {
			// the profile distance is too short to allow us to get to top speed
			// so the profile looks like a triangle instead of a trapezoid
			profile.stopAccelTime = rampUpTime;
			double coastTime = (dist - 2 * rampUpDist) / maxVel;
			profile.startDecelTime = rampUpTime + coastTime;
			profile.totalTime = coastTime + 2 * rampUpTime;
		} else {
			// this is the normal case, where we accelerate to top speed, coast for a bit, then
			// decelerate.
			double totalTime = 2 * std::sqrt(dist / maxAccel);
			profile.stopAccelTime = profile.startDecelTime = totalTime / 2;
			profile.totalTime = totalTime;
		}
		this->profile = profile;
	}

	/**
	 * @brief Get the target position.
	 *
	 * @param currTime The current timestamp.
	 * @return navtypes::Vectord<dim> The target position if hasTarget() is true,
	 * else 0 vector.
	 */
	navtypes::Vectord<dim> getCommand(robot::types::datatime_t currTime) const {
		if (!profile) {
			return navtypes::Vectord<dim>::Zero();
		}

		profile_t profile = this->profile.value();
		navtypes::Vectord<dim> dir = (profile.endPos - profile.startPos).normalized();
		double elapsedTime = util::durationToSec(currTime - profile.startTime);
		if (elapsedTime < 0) {
			return profile.startPos;
		} else if (elapsedTime < profile.stopAccelTime) {
			double dist = 0.5 * maxAccel * std::pow(elapsedTime, 2);
			return profile.startPos + dir * dist;
		} else if (elapsedTime < profile.startDecelTime) {
			// area of trapezoid is average of bases times height
			double dist = (2 * elapsedTime - profile.stopAccelTime) / 2.0 * maxVel;
			return profile.startPos + dir * dist;
		} else if (elapsedTime < profile.totalTime) {
			double distFromEnd = 0.5 * maxAccel * std::pow(profile.totalTime - elapsedTime, 2);
			return profile.endPos - dir * distFromEnd;
		} else {
			return profile.endPos;
		}
	}

	/**
	 * @brief Get the total time it takes to perform the profile.
	 *
	 * @return std::optional<util::dseconds> The total time.
	 * If hasTarget() is false, returns empty optional.
	 */
	std::optional<util::dseconds> getTotalTime() const {
		if (profile.has_value()) {
			return util::dseconds(profile->totalTime);
		} else {
			return {};
		}
	}

private:
	struct profile_t {
		robot::types::datatime_t startTime;
		double stopAccelTime;
		double startDecelTime;
		double totalTime;
		navtypes::Vectord<dim> startPos;
		navtypes::Vectord<dim> endPos;
	};
	double maxVel, maxAccel;
	std::optional<profile_t> profile;
};

/**
 * @brief Trapezoidal velocity profile in a single dimension.
 * @see https://robotacademy.net.au/lesson/1d-trapezoidal-trajectory/
 */
class SingleDimTrapezoidalVelocityProfile {
public:
	/**
	 * @brief Construct a new velocity profile.
	 *
	 * @param maxVel The maximum velocity the profile should attain.
	 * @param maxAccel The maximum acceleration the profile should use.
	 */
	SingleDimTrapezoidalVelocityProfile(double maxVel, double maxAccel);

	/**
	 * @brief Calculate a profile, assuming the mechanism begins and ends at rest.
	 *
	 * @param currTime The current timestamp.
	 * @param startPos The position the profile starts at.
	 * @param endPos The position the profile ends at.
	 */
	void setTarget(robot::types::datatime_t currTime, double startPos, double endPos);

	/**
	 * @brief Check if a profile has been calculated.
	 *
	 * @return bool True iff setTarget() has been called since construction or the last reset.
	 */
	bool hasTarget() const;

	/**
	 * @brief Get the target position.
	 *
	 * @param currTime The current timestamp.
	 * @return double The target position if hasTarget() is true, else 0.
	 */
	double getCommand(robot::types::datatime_t currTime) const;

	/**
	 * @brief Reset the profile and clear any calculated profile.
	 */
	void reset();

	/**
	 * @brief Get the total time it takes to perform the profile.
	 *
	 * @return std::optional<util::dseconds> The total time.
	 * If hasTarget() is false, returns empty optional.
	 */
	std::optional<util::dseconds> getTotalTime() const;

private:
	TrapezoidalVelocityProfile<1> profile;
};

} // namespace control
