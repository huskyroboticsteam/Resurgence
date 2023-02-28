#include "../../src/utils/math.h"

#include <Eigen/Core>
#include <catch2/catch.hpp>

TEST_CASE("Test Numerical Jacobian", "[util][math]") {
	SECTION("Test identity function") {
		auto func = [](const Eigen::Vector3d& x) { return x; };
		Eigen::MatrixXd jacob = util::numericalJacobian(func, Eigen::Vector3d::Random(), 3);
		REQUIRE(jacob.rows() == 3);
		REQUIRE(jacob.cols() == 3);
		REQUIRE((jacob - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() <= 1e-4);
	}
	SECTION("Test inner product with self") {
		auto func = [](const Eigen::Vector3d& x) -> Eigen::Matrix<double, 1, 1> {
			return x.transpose() * x;
		};
		Eigen::Vector3d vec{1.0, 2.0, 3.0};
		Eigen::MatrixXd jacob = util::numericalJacobian(func, vec, 1);
		REQUIRE(jacob.rows() == 1);
		REQUIRE(jacob.cols() == 3);
		REQUIRE((jacob - 2 * vec.transpose()).cwiseAbs().maxCoeff() <= 1e-4);
	}
	SECTION("Test constant function") {
		auto func = [](const Eigen::Vector2d& x) { return Eigen::Vector4d::Ones(); };
		Eigen::Vector2d vec = Eigen::Vector2d::Random();
		Eigen::MatrixXd jacob = util::numericalJacobian(func, vec, 4);
		REQUIRE(jacob.rows() == 4);
		REQUIRE(jacob.cols() == 2);
		REQUIRE((jacob - Eigen::Matrix<double, 4, 2>::Zero()).cwiseAbs().maxCoeff() <= 1e-4);
	}
	SECTION("Test random matrix product") {
		for (int i = 0; i < 20; i++) {
			constexpr int output = 5, input = 10;
			Eigen::Matrix<double, output, input> mat =
				Eigen::Matrix<double, output, input>::Random();
			auto func = [&mat](const Eigen::VectorXd& x) { return mat * x; };
			Eigen::Matrix<double, input, 1> vec = Eigen::Matrix<double, input, 1>::Random();
			Eigen::MatrixXd jacob = util::numericalJacobian(func, vec, output);
			REQUIRE(jacob.rows() == output);
			REQUIRE(jacob.cols() == input);
			REQUIRE((jacob - mat).cwiseAbs().maxCoeff() <= 1e-4);
		}
	}
}