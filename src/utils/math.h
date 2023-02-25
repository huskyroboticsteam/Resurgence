#pragma once

#include <Eigen/Core>

namespace util {

constexpr double epsilon = 1e-5;

Eigen::MatrixXd
numericalJacobian(const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& func,
				  const Eigen::VectorXd& x, int outputDim);

} // namespace util
