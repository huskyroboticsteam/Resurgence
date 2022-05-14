#include "StateSpaceUtil.h"

namespace filters::statespace {

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

NoiseCovMat<-1, -1, -1>::NoiseCovMat(const Eigen::VectorXd& stdDevs, int stateDim,
									 int paramSize)
	: stateDim(stateDim), size(stdDevs.size()), paramDim(paramSize) {
	Eigen::MatrixXd covMat(Eigen::MatrixXd::Zero(stdDevs.size(), stdDevs.size()));
	covMat.diagonal() = stdDevs;
	func = [=](const Eigen::VectorXd& x, const Eigen::VectorXd& param) { return covMat; };
}

NoiseCovMat<-1, -1, -1>::NoiseCovMat(const Eigen::MatrixXd& mat, int stateDim, int paramSize)
	: stateDim(stateDim), size(mat.rows()), paramDim(paramSize) {
	assert(mat.rows() == mat.cols());
	func = [=](const Eigen::VectorXd& x, const Eigen::VectorXd& param) { return mat; };
}

NoiseCovMat<-1, -1, -1>::NoiseCovMat(
	const std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>& func,
	int stateDim, int size, int paramSize)
	: stateDim(stateDim), size(size), paramDim(paramSize), func(func) {}

Eigen::MatrixXd NoiseCovMat<-1, -1, -1>::get(const Eigen::VectorXd& x,
											 const Eigen::VectorXd& param) const {
	assert(x.size() == stateDim && param.size() == paramDim);
	return func(x, param);
}
} // namespace filters::statespace
