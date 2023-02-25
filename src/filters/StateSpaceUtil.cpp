#include "StateSpaceUtil.h"

namespace filters::statespace {

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
