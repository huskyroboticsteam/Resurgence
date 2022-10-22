#pragma once

#include "../world_interface/data.h"

#include <array>
#include <cmath>
#include <optional>
#include <vector>

#include <Eigen/Core>

namespace control {

// TODO: change to use timestamps instead of elapsed time (maybe use both)

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
	 * @brief Calculate a profile.
	 *
	 * @param startPos The position the profile starts at.
	 * @param endPos The position the profile ends at.
	 */
	void setTarget(double startPos, double endPos);

	/**
	 * @brief Check if a profile has been calculated.
	 *
	 * @return bool True iff setTarget() has been called since construction or the last reset.
	 */
	bool hasTarget() const;

	/**
	 * @brief Get the target position.
	 *
	 * @param elapsedTime Time elapsed, in seconds, since the profile was constructed.
	 * @return double The target position if hasTarget() is true, else 0.
	 */
	double getCommand(double elapsedTime) const;

	/**
	 * @brief Reset the profile and clear any calculated profile.
	 */
	void reset();

private:
	struct profile_t {
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

	bool hasTarget() {
		return std::any_of(profiles.begin(), profiles.end(),
						   [](const auto& p) { return p.hasTarget(); });
	}

	void setTarget(const std::array<double, dim>& startPos,
				   const std::array<double, dim>& endPos) {
		for (int i = 0; i < dim; i++) {
			profiles[i].setTarget(startPos[i], endPos[i]);
		}
	}

	void setTarget(const Eigen::Matrix<double, dim, 1>& startPos,
				   const Eigen::Matrix<double, dim, 1>& endPos) {
		for (int i = 0; i < dim; i++) {
			profiles[i].setTarget(startPos[i], endPos[i]);
		}
	}

	// returns zero vector if no target
	Eigen::Matrix<double, dim, 1> getCommand(double elapsedTime) const {
		Eigen::Matrix<double, dim, 1> ret;
		for (int i = 0; i < dim; i++) {
			ret[i] = profiles[i].getCommand(elapsedTime);
		}
		return ret;
	}

private:
	std::vector<SingleDimTrapezoidalVelocityProfile> profiles;
};

} // namespace control
