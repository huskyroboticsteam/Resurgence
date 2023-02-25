#pragma once

#include <Eigen/Core>

namespace util {

/**
 * @brief A small positive value, near zero.
 */
constexpr double epsilon = 1e-5;

/**
 * @brief Estimate the jacobian of a multivariate function using finite differences.
 *
 * @param func The function to estimate the jacobian of.
 * @param x The point around which to calculate the jacobian.
 * @param outputDim The dimension of the output space of @p func.
 * @return Eigen::MatrixXd An n-by-m matrix where n is the output dimension and m is the input
 * dimension of @p func.
 */
Eigen::MatrixXd
numericalJacobian(const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& func,
				  const Eigen::VectorXd& x, int outputDim);

} // namespace util
