#include "../../src/filters/FullPoseEstimator.h"

#include "../../src/Constants.h"
#include "../../src/filters/StateSpaceUtil.h"
#include "../../src/kinematics/DiffDriveKinematics.h"
#include "../../src/navtypes.h"

#include <catch2/catch.hpp>

using namespace filters;
using namespace navtypes;
using navtypes::Vectord;

TEST_CASE("FullPoseEstimator", "[filters]") {
	FullPoseEstimator estimator({0.01, 0.01}, Constants::EFF_WHEEL_BASE, 0.1, {0.01, 0.01},
								0.01);

	SECTION("Correct Only") {
		for (int i = 0; i < 10; i++) {
			estimator.reset(pose_t::Zero());
			Eigen::Vector2d gpsPos = Eigen::Vector2d::Random();
			point_t p = point_t::Ones();
			p.topRows<2>() = gpsPos;
			estimator.correctGPS(p);

			Vectord<1> heading = Vectord<1>::Random();
			estimator.correctHeading(heading(0));

			pose_t pose;
			pose.topRows<2>() = gpsPos;
			pose.bottomRows<1>() = heading;

			REQUIRE((pose - estimator.getPose()).array().abs().maxCoeff() <= 1e-3);
		}
	}

	SECTION("Predict Only") {
		DiffDriveKinematics kinematics(Constants::EFF_WHEEL_BASE);
		estimator.reset(pose_t::Zero());

		pose_t pose = estimator.getPose();
		for (int i = 0; i < 10; i++) {
			Eigen::Vector2d action = Eigen::Vector2d::Random();
			wheelvel_t vels{.lVel = action(0), .rVel = action(1)};
			pose = kinematics.getNextPose(vels, pose, 0.1);
			pose_t robotVel = kinematics.wheelVelToRobotVel(vels.lVel, vels.rVel);
			estimator.predict(robotVel(2), robotVel(0));
			REQUIRE((pose - estimator.getPose()).array().abs().maxCoeff() <= 1e-3);
		}
	}
}
