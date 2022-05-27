#pragma once

// define TRAP_PROF_NO_EIGEN to disable eigen

#include "../world_interface/data.h"

#include <array>
#include <cmath>
#include <optional>
#include <vector>

#ifndef TRAP_PROF_NO_EIGEN
#include <Eigen/Core>
#endif

namespace control {

class SingleDimTrapezoidalVelocityProfile {
public:
	SingleDimTrapezoidalVelocityProfile(double maxVel, double maxAccel);

	void setTarget(double startPos, double endPos);

	// returns target position
	double getCommand(double elapsedTime) const;

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

	void setTarget(const std::array<double, dim>& startPos,
				   const std::array<double, dim>& endPos) {
		for (int i = 0; i < dim; i++) {
			profiles[i].setTarget(startPos[i], endPos[i]);
		}
	}

#ifndef TRAP_PROF_NO_EIGEN
	void setTarget(const Eigen::Matrix<double, dim, 1>& startPos,
				   const Eigen::Matrix<double, dim, 1>& endPos) {
		for (int i = 0; i < dim; i++) {
			profiles[i].setTarget(startPos[i], endPos[i]);
		}
	}

	Eigen::Matrix<double, dim, 1> getCommand(double elapsedTime) const {
		Eigen::Matrix<double, dim, 1> ret;
		for (int i = 0; i < dim; i++) {
			ret[i] = profiles[i].getCommand(elapsedTime);
		}
		return ret;
	}
#else
	std::array<double, dim> getCommand(double elapsedTime) const {
		std::array<double, dim> ret;
		for (int i = 0; i < dim; i++) {
			ret[i] = profiles[i].getCommand(elapsedTime);
		}
		return ret;
	}
#endif

private:
	std::vector<SingleDimTrapezoidalVelocityProfile> profiles;
};

} // namespace control
