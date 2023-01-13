#include <catch2/catch.hpp>
#include <cmath>
#include <cstdio>

#include "../../src/commands/CommandBase.h"
#include "../../src/commands/DriveToWaypointCommand.h"

void assertApprox(double expectedThetaVel, double expectedXVel,
                  const commands::command_t& actual, double angle = 1e-5,
                  double vel = 1e-5) {
  bool velEqual = abs(expectedThetaVel - actual.thetaVel) <= angle;
  bool angleEqual = abs(expectedXVel - actual.xVel) <= vel;

  if (velEqual && angleEqual) {
    SUCCEED();
  } else {
    printf(
        "Expected (thetaVel,xVel)=(%.5f,%.5f), "
        "Actual (thetaVel,xVel)=(%.5f,%.5f)\n",
        expectedThetaVel, expectedXVel, actual.thetaVel, actual.xVel);
    FAIL();
  }
}

TEST_CASE("DriveToWaypointCommand Test") {
  double atan_4_3 = 0.927295218002;  // value of atan(4/3) in rads.
  double theta_kp = 3.5;

  // sample target is at position (3,4).
  navtypes::point_t target(3.0, 4.0, 1.5);

  // instantiate DriveToWaypointCommand with sample target.
  commands::DriveToWaypointCommand cmd(target, theta_kp, 4.0, 2.0, 0.25);

  // test robot at origin, target at (3,4)
  navtypes::pose_t origin(0.0, 0.0, 0.0);
  cmd.setState(origin);
  assertApprox(theta_kp * atan_4_3, 4.0, cmd.getOutput());

  // test robot on midpoint along line segment between origin and target.
  navtypes::pose_t similar(1.5, 2.0, 0.0);
  cmd.setState(similar);
  assertApprox(theta_kp * atan_4_3, 4.0, cmd.getOutput());

  // test robot and target parallel along X-axis, robot directly facing target.
  navtypes::pose_t parallelX(0.0, 4.0, 0.0);
  cmd.setState(parallelX);
  assertApprox(0.0, 4.0, cmd.getOutput());

  // test robot parallel to target along X-axis, robot facing opposite target.
  navtypes::pose_t aparallelX(6.0, 4.0, 0.0);
  cmd.setState(aparallelX);
  assertApprox(theta_kp * M_PI, 4.0, cmd.getOutput());

  // test robot and target parallel along Y-axis, robot directly facing target
  navtypes::pose_t parallelY(3.0, 0.0, 0.0);
  cmd.setState(parallelY);
  assertApprox(theta_kp * M_PI / 2.0, 4.0, cmd.getOutput());

  // test robot and target parallel along Y-axis, robot facing opposite target.
  navtypes::pose_t aparallelY(3.0, 0.0, -M_PI / 2.0);
  cmd.setState(aparallelY);
  assertApprox(theta_kp * M_PI, 4.0, cmd.getOutput());

  // test robot uses slow drive speed if within done threshold
  // (in this case, done threshold is 25 cm)
  navtypes::pose_t close_enough(3.0 - (sqrt(9.0 / 25.0) / 16.0),
                                4.0 - (sqrt(9.0 / 25.0) / 12.0), 0.0);
  cmd.setState(close_enough);
  assertApprox(theta_kp * atan_4_3, 2.0, cmd.getOutput());
}