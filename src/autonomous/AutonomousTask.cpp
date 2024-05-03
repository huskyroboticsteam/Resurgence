#include "AutonomousTask.h"

#include "../Constants.h"
#include "../commands/DriveToWaypointCommand.h"
#include "../utils/transform.h"
#include "../world_interface/world_interface.h"

#include <loguru.hpp>

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
	commands::DriveToWaypointCommand cmd(_waypoint_coords, THETA_KP, DRIVE_VEL,
										 SLOW_DRIVE_THRESHOLD, DONE_THRESHOLD,
										 CLOSE_TO_TARGET_DUR_VAL);

	DiffDriveKinematics diffDriveKinematics(Constants::EFF_WHEEL_BASE);

	auto sleepUntil = std::chrono::steady_clock().now();
	while (!cmd.isDone()) {
		auto latestGPS = robot::readGPS();
		auto latestHeading = robot::readIMUHeading();

		if (latestGPS.isFresh(2000ms) && latestHeading.isFresh(2000ms)) {
			LOG_SCOPE_F(INFO, "AutoNav");
			auto gpsData = latestGPS.getData();
			navtypes::pose_t latestPos(gpsData.x(), gpsData.y(), latestHeading.getData());
			cmd.setState(latestPos, robot::types::dataclock::now());
			commands::command_t output = cmd.getOutput();
			auto scaledVels = diffDriveKinematics.ensureWithinWheelSpeedLimit(
				DiffDriveKinematics::PreferredVelPreservation::PreferThetaVel, output.xVel,
				output.thetaVel, Constants::MAX_WHEEL_VEL);
			navtypes::point_t relTarget = util::toTransform(latestPos) * _waypoint_coords;
			LOG_F(INFO, "Target: (%lf, %lf)", _waypoint_coords(0), _waypoint_coords(1));
			LOG_F(INFO, "CurPos: (%lf, %lf)", latestPos(0), latestPos(1));
			LOG_F(INFO, "Relative Target: (%lf, %lf)", relTarget(0), relTarget(1));
			LOG_F(INFO, "curHeading: %lf", latestPos(2));
			LOG_F(INFO, "thetaVel: %lf", scaledVels(2));
			LOG_F(INFO, "xVel: %lf", scaledVels(0));
			robot::setCmdVel(scaledVels(2), scaledVels(0));
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
