#pragma once

#include "../world_interface/data.h"
#include "TrapezoidalVelocityProfile.h"

#include <array>
#include <functional>
#include <optional>

#include <Eigen/Core>

namespace control {

template <int outputDim, int inputDim> class JacobianController {
private:
	template <int d> using Vectord = Eigen::Matrix<double, d, 1>;
	template <int r, int c> using Matrixd = Eigen::Matrix<double, r, c>;

public:
	JacobianController(
		const std::function<Vectord<outputDim>(const Vectord<inputDim>&)>& kinematicsFunc,
		const std::function<Matrixd<outputDim, inputDim>(const Vectord<inputDim>&)>&
			jacobianFunc,
		const std::array<double, outputDim>& maxVels,
		const std::array<double, outputDim>& maxAccels)
		: kinematicsFunc(kinematicsFunc),
		  jacobianFunc(jacobianFunc ? jacobianFunc : jacobianFunc),
		  velocityProfile(maxVels, maxAccels) {
		// TODO: numerically approximate with statespace::numericalJacobian
		assert(kinematicsFunc && jacobianFunc);
	}

	Vectord<inputDim> getCommand(double elapsedTime, const Vectord<inputDim>& currPos) const {
		Vectord<outputDim> currTarget = velocityProfile.getCommand(elapsedTime);
		Vectord<outputDim> currPosOutput = kinematicsFunc(currPos);
		Vectord<outputDim> outputPosDiff = currTarget - currPosOutput;
		Matrixd<outputDim, inputDim> jacobian = jacobianFunc(currPos);
		Vectord<inputDim> inputPosDiff = jacobian.colPivHouseholderQR().solve(outputPosDiff);
		return currPos + inputPosDiff;
	}

	void setTarget(const Vectord<inputDim>& currPos, const Vectord<outputDim>& target) {
		Vectord<outputDim> currPosOutput = kinematicsFunc(currPos);
		velocityProfile.setTarget(currPosOutput, target);
	}

private:
	std::function<Vectord<outputDim>(const Vectord<inputDim>&)> kinematicsFunc;
	std::function<Eigen::Matrix<double, outputDim, inputDim>(
		const Eigen::Matrix<double, inputDim, 1>&)>
		jacobianFunc;
	TrapezoidalVelocityProfile<outputDim> velocityProfile;
};

} // namespace control
