#include "AutonomousTask.h"

#include "../Constants.h"
#include "../Globals.h"
#include "../commands/DriveToWaypointCommand.h"
#include "../world_interface/world_interface.h"

using namespace std::chrono_literals;

namespace autonomous {

AutonomousTask::AutonomousTask(){};

AutonomousTask::~AutonomousTask() {
	if (_autonomous_task_thread.joinable()) {
		_autonomous_task_thread.join();
	}
}

void AutonomousTask::start(const navtypes::point_t& waypointCoords) {
	if (_autonomous_task_thread.joinable()) {
		kill();
	}

	_waypoint_coords = waypointCoords;
	_kill_called = false;
	_autonomous_task_thread = std::thread(&autonomous::AutonomousTask::navigate, this);
}

void AutonomousTask::navigate() {
	commands::DriveToWaypointCommand cmd(_waypoint_coords, THETA_KP, DRIVE_VEL, SLOW_DRIVE_VEL,
										 DONE_THRESHOLD, CLOSE_TO_TARGET_DUR_VAL);

	auto sleepUntil = std::chrono::steady_clock().now();
	while (!cmd.isDone()) {
		auto latestGPS = robot::readGPS();
		auto latestHeading = robot::readIMUHeading();

		if (latestGPS.isFresh(2000ms) && latestHeading.isFresh(2000ms)) {
			auto gpsData = latestGPS.getData();
			navtypes::pose_t latestPos(gpsData.x(), gpsData.y(), latestHeading);
			cmd.setState(latestPos, robot::types::dataclock::now());
			commands::command_t output = cmd.getOutput();
			robot::setCmdVel(output.thetaVel, output.xVel);
		}

		std::unique_lock autonomousTaskLock(_autonomous_task_mutex);
		sleepUntil += 500ms;
		// Wait 500ms or return out of while loop if kill called
		if (_autonomous_task_cv.wait_until(autonomousTaskLock, sleepUntil,
										   [&] { return _kill_called; })) {
			return;
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
