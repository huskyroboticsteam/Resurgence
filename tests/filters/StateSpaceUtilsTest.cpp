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
