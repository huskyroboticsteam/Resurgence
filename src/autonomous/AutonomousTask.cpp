#include "AutonomousTask.h"

#include "../Constants.h"
#include "../commands/DriveToWaypointCommand.h"
#include "../control_interface.h"
#include "../utils/transform.h"
#include "../world_interface/world_interface.h"

#include <loguru.hpp>

#include <queue>

using namespace std::chrono_literals;
using namespace Constants::autonomous;

namespace autonomous {

AutonomousTask::AutonomousTask(){};

AutonomousTask::AutonomousTask() {
	if (_autonomous_task_thread.joinable()) {
		_autonomous_task_thread.join();
	}
}

void AutonomousTask::start(std::queue<navtypes::point_t> waypoints) {
	if (_autonomous_task_thread.joinable()) {
		kill();
	}
	_waypoint_queue = std::move(waypoints);
	_kill_called = false;
	_autonomous_task_thread = std::thread(&autonomous::AutonomousTask::navigate, this);
}



void AutonomousTask::navigate() {
	kinematics::DiffDriveKinematics diffDriveKinematics(Constants::EFF_WHEEL_BASE);

	while (!_waypoint_queue.empty() && !_kill_called) {
		_waypoint_coords = _waypoint_queue.front(); // set current waypoint
		_waypoint_queue.pop();

		commands::DriveToWaypointCommand cmd(_waypoint_coords, THETA_KP, DRIVE_VEL,
											 SLOW_DRIVE_THRESHOLD, DONE_THRESHOLD,
											 CLOSE_TO_TARGET_DUR_VAL);

		auto sleepUntil = std::chrono::steady_clock().now();
		while (!cmd.isDone() && !_kill_called) {
			auto latestGPS = robot::readGPS();
			auto latestHeading = robot::readIMUHeading();

			if (latestGPS.isFresh(2000ms) && latestHeading.isFresh(2000ms)) {
				LOG_SCOPE_F(INFO, "AutoNav");
				auto gpsData = latestGPS.getData();
				navtypes::pose_t latestPos(gpsData.x(), gpsData.y(), latestHeading.getData());
				cmd.setState(latestPos, robot::types::dataclock::now());

				commands::command_t output = cmd.getOutput();
				auto scaledVels = diffDriveKinematics.ensureWithinWheelSpeedLimit(
					kinematics::DiffDriveKinematics::PreferredVelPreservation::PreferThetaVel,
					output.xVel, output.thetaVel, Constants::MAX_WHEEL_VEL);

				navtypes::point_t relTarget = util::toTransform(latestPos) * _waypoint_coords;
				LOG_F(INFO, "Relative Target: (%lf, %lf)", relTarget(0), relTarget(1));
				LOG_F(INFO, "thetaVel: %lf", scaledVels(2));
				LOG_F(INFO, "xVel: %lf", scaledVels(0));
				robot::setCmdVel(scaledVels(2), scaledVels(0));
			}

			std::unique_lock autonomousTaskLock(_autonomous_task_mutex);
			sleepUntil += 500ms;
			if (_autonomous_task_cv.wait_until(autonomousTaskLock, sleepUntil,
											   [&] { return _kill_called; })) {
				return;
			}
		}
	}
}


void AutonomousTask::kill() {
	{
		std::lock_guard lock(_autonomous_task_mutex);
		_kill_called = true;
	}

	_autonomous_task_cv.notify_all();

	if (_autonomous_task_thread.joinable()) {
		_autonomous_task_thread.join();
	}
}

} // namespace autonomous
