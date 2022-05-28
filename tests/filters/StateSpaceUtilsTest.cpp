#include <catch2/catch.hpp>

#include "../../src/filters/StateSpaceUtil.h"

using namespace filters;

TEST_CASE("Test Continuous to Discrete System Model", "[statespace][filters]") {
	// examples pulled from appendix of https://doi.org/10.1016/0307-904X(80)90177-8
	Eigen::Matrix2d A, B;
	A << 1, 2, 3, -4;
	B << 2, 0, 1, 1;

	statespace::continuousToDiscrete(A, B, 0.25);

	Eigen::Matrix2d discA, discB; // these are called G and H in the paper
	discA << 1.45412, 0.38920, 0.58381, 0.48111;
	discB << 0.64872, 0.05190, 0.32437, 0.16865;

	CHECK((A-discA).cwiseAbs().sum() == Approx(0).margin(1e-4));
	CHECK((B-discB).cwiseAbs().sum() == Approx(0).margin(1e-4));
}

TEST_CASE("Test Discrete to Continuous System Model", "[statespace][filters]") {
	Eigen::Matrix2d A, B;
	A << 1.45412, 0.38920, 0.58381, 0.48111;
	B << 0.64872, 0.05190, 0.32437, 0.16865;

	statespace::discreteToContinuous(A, B, 0.25);

	Eigen::Matrix2d contA, contB;
	contA << 1, 2, 3, -4;
	contB << 2, 0, 1, 1;

	CHECK((A-contA).cwiseAbs().sum() == Approx(0).margin(1e-4));
	CHECK((B-contB).cwiseAbs().sum() == Approx(0).margin(1e-4));
}

TEST_CASE("Test Numerical Jacobian", "[statespace][filters]") {
	SECTION("Test identity function") {
		auto func = [](const Eigen::Vector3d& x) { return x; };
		Eigen::MatrixXd jacob = statespace::numericalJacobian(func, Eigen::Vector3d::Random(), 3);
		REQUIRE(jacob.rows() == 3);
		REQUIRE(jacob.cols() == 3);
		REQUIRE((jacob - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() <= 1e-4);
	}
	SECTION("Test inner product with self") {
		auto func = [](const Eigen::Vector3d& x) { return x.transpose() * x; };
		Eigen::Vector3d vec{1.0, 2.0, 3.0};
		Eigen::MatrixXd jacob = statespace::numericalJacobian(func, vec, 1);
		REQUIRE(jacob.rows() == 1);
		REQUIRE(jacob.cols() == 3);
		REQUIRE((jacob - 2 * vec.transpose()).cwiseAbs().maxCoeff() <= 1e-4);
	}
	SECTION("Test constant function") {
		auto func = [](const Eigen::Vector2d& x) { return Eigen::Vector4d::Ones(); };
		Eigen::Vector2d vec = Eigen::Vector2d::Random();
		Eigen::MatrixXd jacob = statespace::numericalJacobian(func, vec, 4);
		REQUIRE(jacob.rows() == 4);
		REQUIRE(jacob.cols() == 2);
		REQUIRE((jacob - Eigen::Matrix<double, 4, 2>::Zero()).cwiseAbs().maxCoeff() <= 1e-4);
	}
	SECTION("Test random matrix product") {
		for (int i = 0; i < 20; i++) {
			constexpr int output = 5, input = 10;
			Eigen::Matrix<double, output, input> mat = Eigen::Matrix<double, output, input>::Random();
			auto func = [&mat](const Eigen::VectorXd& x) { return mat * x; };
			Eigen::Matrix<double, input, 1> vec = Eigen::Matrix<double, input, 1>::Random();
			Eigen::MatrixXd jacob = statespace::numericalJacobian(func, vec, output);
			REQUIRE(jacob.rows() == output);
			REQUIRE(jacob.cols() == input);
			REQUIRE((jacob - mat).cwiseAbs().maxCoeff() <= 1e-4);
		}
	}
}
