#pragma once

#include "../world_interface/data.h"

#include <array>
#include <cmath>
#include <optional>
#include <vector>

#include <Eigen/Core>

namespace control {

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

private:
	struct profile_t {
		robot::types::datatime_t startTime;
		double stopAccelTime;
		double startDecelTime;
		double totalTime;
		double startPos;
		double endPos;
	};
	double maxVel, maxAccel;
	std::optional<profile_t> profile;
};

template <int dim> class TrapezoidalVelocityProfile {
public:
	TrapezoidalVelocityProfile(const std::array<double, dim>& maxVels,
							   const std::array<double, dim>& maxAccels) {
		for (int i = 0; i < dim; i++) {
			profiles.emplace_back(maxVels[i], maxAccels[i]);
		}
	}

	void reset() {
		for (auto& profile : profiles) {
			profile.reset();
		}
	}

	bool hasTarget() const {
		return std::any_of(profiles.begin(), profiles.end(),
						   [](const auto& p) { return p.hasTarget(); });
	}

	void setTarget(robot::types::datatime_t currTime,
				   const std::array<double, dim>& startPos,
				   const std::array<double, dim>& endPos) {
		for (int i = 0; i < dim; i++) {
			profiles[i].setTarget(currTime, startPos[i], endPos[i]);
		}
	}

	void setTarget(robot::types::datatime_t currTime,
				   const Eigen::Matrix<double, dim, 1>& startPos,
				   const Eigen::Matrix<double, dim, 1>& endPos) {
		for (int i = 0; i < dim; i++) {
			profiles[i].setTarget(currTime, startPos[i], endPos[i]);
		}
	}

	// returns zero vector if no target
	Eigen::Matrix<double, dim, 1> getCommand(robot::types::datatime_t currTime) const {
		Eigen::Matrix<double, dim, 1> ret;
		for (int i = 0; i < dim; i++) {
			ret[i] = profiles[i].getCommand(currTime);
		}
		return ret;
	}

private:
	std::vector<SingleDimTrapezoidalVelocityProfile> profiles;
};

} // namespace control
