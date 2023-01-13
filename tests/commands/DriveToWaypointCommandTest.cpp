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

const double ATAN_4_3 = 0.927295218002; // value of atan(4/3) in rad.
const double THETA_KP = 3.5;
const double NORMAL_DRIVE_SPEED = 4.0;
const double SLOW_DRIVE_SPEED = 2.0;
const double DONE_THRESHOLD = 0.25;

TEST_CASE("DriveToWaypointCommand Test") {
  // sample target is at position (3,4).
  navtypes::point_t target(3.0, 4.0, 1.5);

  // instantiate DriveToWaypointCommand with sample target.
  commands::DriveToWaypointCommand cmd(target, THETA_KP, NORMAL_DRIVE_SPEED,
                                       SLOW_DRIVE_SPEED, DONE_THRESHOLD);

  // test robot at origin, target at (3,4)
  navtypes::pose_t origin(0.0, 0.0, 0.0);
  cmd.setState(origin);
  assertApprox(THETA_KP * ATAN_4_3, NORMAL_DRIVE_SPEED, cmd.getOutput());

  // test robot on midpoint along line segment between origin and target.
  navtypes::pose_t similar(1.5, 2.0, 0.0);
  cmd.setState(similar);
  assertApprox(THETA_KP * ATAN_4_3, NORMAL_DRIVE_SPEED, cmd.getOutput());

  // test robot and target parallel along X-axis, robot directly facing target.
  navtypes::pose_t parallelX(0.0, 4.0, 0.0);
  cmd.setState(parallelX);
  assertApprox(0.0, NORMAL_DRIVE_SPEED, cmd.getOutput());

  // test robot parallel to target along X-axis, robot facing opposite target.
  navtypes::pose_t aparallelX(6.0, 4.0, 0.0);
  cmd.setState(aparallelX);
  assertApprox(THETA_KP * M_PI, NORMAL_DRIVE_SPEED, cmd.getOutput());

  // test robot and target parallel along Y-axis, robot directly facing target
  navtypes::pose_t parallelY(3.0, 0.0, 0.0);
  cmd.setState(parallelY);
  assertApprox(THETA_KP * M_PI / 2.0, NORMAL_DRIVE_SPEED, cmd.getOutput());

  // test robot and target parallel along Y-axis, robot facing opposite target.
  navtypes::pose_t aparallelY(3.0, 0.0, -M_PI / 2.0);
  cmd.setState(aparallelY);
  assertApprox(THETA_KP * M_PI, NORMAL_DRIVE_SPEED, cmd.getOutput());

  // test robot uses slow drive speed if within done threshold
  // (in this case, done threshold is 25 cm)
  navtypes::pose_t close_enough(3.0 - (3.0 / (5.0 * 16.0)),
                                4.0 - (3.0 / (5.0 * 12.0)), 0.0);
  cmd.setState(close_enough);
  assertApprox(THETA_KP * ATAN_4_3, SLOW_DRIVE_SPEED, cmd.getOutput());
}