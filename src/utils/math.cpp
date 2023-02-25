#include "math.h"

namespace util {

Eigen::MatrixXd
numericalJacobian(const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& func,
				  const Eigen::VectorXd& x, int outputDim) {
	int inputDim = x.size();
	Eigen::MatrixXd jacobian(outputDim, inputDim);
	for (int i = 0; i < inputDim; i++) {
		Eigen::VectorXd delta = Eigen::VectorXd::Zero(inputDim);
		delta[i] = epsilon;
		Eigen::VectorXd derivative = (func(x + delta) - func(x - delta)) / (2 * epsilon);
		jacobian.col(i) = derivative;
	}
	return jacobian;
}

} // namespace util
