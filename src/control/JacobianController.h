#pragma once

#include "../filters/StateSpaceUtil.h"
#include "../world_interface/data.h"
#include "TrapezoidalVelocityProfile.h"

#include <array>
#include <functional>
#include <optional>

#include <Eigen/Core>

namespace control {

template <int outputDim, int inputDim> class JacobianController {
private:
	template <int dim> using Vectord = filters::statespace::Vectord<dim>;
	template <int rows, int cols> using Matrixd = filters::statespace::Matrixd<rows, cols>;

public:
	JacobianController(
		const std::function<Vectord<outputDim>(const Vectord<inputDim>&)>& kinematicsFunc,
		const std::function<Matrixd<outputDim, inputDim>(const Vectord<inputDim>&)>&
			jacobianFunc,
		const std::array<double, outputDim>& maxVels,
		const std::array<double, outputDim>& maxAccels)
		: kinematicsFunc(kinematicsFunc),
		  jacobianFunc(jacobianFunc
						   ? jacobianFunc
						   : std::bind(filters::statespace::numericalJacobian, kinematicsFunc,
									   std::placeholders::_1, outputDim)),
		  velocityProfile(maxVels, maxAccels) {
		assert(this->kinematicsFunc);
		assert(this->jacobianFunc);
	}

	Vectord<inputDim> getCommand(double elapsedTime, const Vectord<inputDim>& currPos) const {
		double unused;
		return getCommand(elapsedTime, currPos, unused);
	}

	Vectord<inputDim> getCommand(double elapsedTime, const Vectord<inputDim>& currPos,
								 double& cosineSim) const {
		if (!velocityProfile.hasTarget()) {
			return currPos;
		}

		Vectord<outputDim> currTarget = velocityProfile.getCommand(elapsedTime);
		Vectord<outputDim> currPosOutput = kinematicsFunc(currPos);
		Vectord<outputDim> outputPosDiff = currTarget - currPosOutput;
		Matrixd<outputDim, inputDim> jacobian = jacobianFunc(currPos);
		Vectord<inputDim> inputPosDiff = jacobian.colPivHouseholderQR().solve(outputPosDiff);

		Vectord<outputDim> trueOutputPosDiff = jacobian * inputPosDiff;
		cosineSim = outputPosDiff.dot(trueOutputPosDiff) /
					(outputPosDiff.norm() * trueOutputPosDiff.norm());

		return currPos + inputPosDiff;
	}

	void setTarget(const Vectord<inputDim>& currPos, const Vectord<outputDim>& target) {
		Vectord<outputDim> currPosOutput = kinematicsFunc(currPos);
		velocityProfile.setTarget(currPosOutput, target);
	}

	bool hasTarget() {
		return velocityProfile.hasTarget();
	}

	void reset() {
		velocityProfile.reset();
	}

private:
	std::function<Vectord<outputDim>(const Vectord<inputDim>&)> kinematicsFunc;
	std::function<Eigen::Matrix<double, outputDim, inputDim>(
		const Eigen::Matrix<double, inputDim, 1>&)>
		jacobianFunc;
	TrapezoidalVelocityProfile<outputDim> velocityProfile;
};

} // namespace control
