#include <catch2/catch.hpp>
#include <cmath>
#include <cstdio>

#include "../../src/commands/CommandBase.h"
#include "../../src/commands/DriveToWaypointCommand.h"

using namespace robot::types;
using namespace std::chrono_literals;

namespace {
const double ATAN_4_3 = 0.927295218002;  // value of atan(4/3) in rad.
const double THETA_KP = 3.5;
const double NORMAL_DRIVE_SPEED = 4.0;
const double SLOW_DRIVE_SPEED = 2.0;
const double DONE_THRESHOLD = 0.25;
const util::dseconds CLOSE_TO_TARGET_DUR_VAL = 1s;
const util::dseconds CLOSE_TO_TARGET_DUR_ALTVAL = 2s;

void assertApprox(double expectedThetaVel, double expectedXVel,
                  const commands::command_t& actual, double angleThresh = 1e-5,
                  double velThresh = 1e-5) {
  REQUIRE_THAT(actual.thetaVel,
               Catch::WithinAbs(expectedThetaVel, angleThresh));
  REQUIRE_THAT(actual.xVel, Catch::WithinAbs(expectedXVel, velThresh));
}

}  // namespace

TEST_CASE("DriveToWaypointCommand Test") {
  // sample target is at position (3,4).
  navtypes::point_t target(3.0, 4.0, 1.5);

  datatime_t startTime = dataclock::now();

  // instantiate DriveToWaypointCommand with sample target.
  commands::DriveToWaypointCommand cmd(target, THETA_KP, NORMAL_DRIVE_SPEED,
                                       SLOW_DRIVE_SPEED, DONE_THRESHOLD,
                                       CLOSE_TO_TARGET_DUR_VAL);

  // test robot at origin, target at (3,4)
  navtypes::pose_t origin(0.0, 0.0, 0.0);
  cmd.setState(origin, startTime);
  assertApprox(THETA_KP * ATAN_4_3, NORMAL_DRIVE_SPEED, cmd.getOutput());
  REQUIRE_FALSE(cmd.isDone());

  // test robot on midpoint along line segment between origin and target.
  navtypes::pose_t similar(1.5, 2.0, 0.0);
  cmd.setState(similar, startTime);
  assertApprox(THETA_KP * ATAN_4_3, NORMAL_DRIVE_SPEED, cmd.getOutput());
  REQUIRE_FALSE(cmd.isDone());

  // test robot and target parallel along X-axis, robot directly facing target.
  navtypes::pose_t parallelX(0.0, 4.0, 0.0);
  cmd.setState(parallelX, startTime);
  assertApprox(0.0, NORMAL_DRIVE_SPEED, cmd.getOutput());
  REQUIRE_FALSE(cmd.isDone());

  // test robot parallel to target along X-axis, robot facing opposite target.
  navtypes::pose_t aparallelX(6.0, 4.0, 0.0);
  cmd.setState(aparallelX, startTime);
  assertApprox(THETA_KP * M_PI, NORMAL_DRIVE_SPEED, cmd.getOutput());
  REQUIRE_FALSE(cmd.isDone());

  // test robot and target parallel along Y-axis, robot directly facing target
  navtypes::pose_t parallelY(3.0, 0.0, 0.0);
  cmd.setState(parallelY, startTime);
  assertApprox(THETA_KP * M_PI / 2.0, NORMAL_DRIVE_SPEED, cmd.getOutput());
  REQUIRE_FALSE(cmd.isDone());

  // test robot and target parallel along Y-axis, robot facing opposite target.
  navtypes::pose_t aparallelY(3.0, 0.0, -M_PI / 2.0);
  cmd.setState(aparallelY, startTime);
  assertApprox(THETA_KP * M_PI, NORMAL_DRIVE_SPEED, cmd.getOutput());
  REQUIRE_FALSE(cmd.isDone());

  // test robot uses slow drive speed if within double the done threshold
  // (in this case, done threshold is 25 cm)
  navtypes::pose_t within_2d(2.7, 3.6, 0.0);
  cmd.setState(within_2d, startTime);
  assertApprox(THETA_KP * ATAN_4_3, SLOW_DRIVE_SPEED, cmd.getOutput());
  REQUIRE_FALSE(cmd.isDone());

  // Ensure cmd.isDone() behaves correctly.
  // set robot position to within distance threshold.
  navtypes::pose_t within_d(2.85, 3.81, 0.0);
  cmd.setState(within_d, startTime);
  REQUIRE_FALSE(cmd.isDone());
  cmd.setState(within_d, startTime + 1s);
  REQUIRE(cmd.isDone());
}

TEST_CASE("DriveToWaypointCommand Test Alternate closeToTargetDur") {
  navtypes::point_t target(3.0, 4.0, 1.5);

  datatime_t startTime = dataclock::now();

  // instantiate DriveToWaypointCommand with sample target.
  commands::DriveToWaypointCommand cmd(target, THETA_KP, NORMAL_DRIVE_SPEED,
                                       SLOW_DRIVE_SPEED, DONE_THRESHOLD,
                                       CLOSE_TO_TARGET_DUR_ALTVAL);

  // Ensure cmd.isDone() behaves correctly.
  // set robot position to within distance threshold.
  navtypes::pose_t within_d(2.85, 3.81, 0.0);
  cmd.setState(within_d, startTime);
  REQUIRE_FALSE(cmd.isDone());
  cmd.setState(within_d, startTime + 2s);
  REQUIRE(cmd.isDone());
}
