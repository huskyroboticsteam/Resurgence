#include "AutonomousTask.h"

#include "../Constants.h"
#include "../commands/DriveToWaypointCommand.h"
#include "../control_interface.h"
#include "../utils/transform.h"
#include "../world_interface/world_interface.h"

#include <loguru.hpp>

using namespace std::chrono_literals;
using namespace Constants::autonomous;

using nlohmann::json;

namespace autonomous {

AutonomousTask::AutonomousTask(net::websocket::SingleClientWSServer& server): _server(server) {};

AutonomousTask::~AutonomousTask() {
	if (_autonomous_task_thread.joinable()) {
		_autonomous_task_thread.join();
	}
}

void AutonomousTask::start(const navtypes::points_t& waypointCoords) {
	if (_autonomous_task_thread.joinable()) {
		kill();
	}

	_waypoint_coords_list = waypointCoords;
	_kill_called = false;
	
	_autonomous_task_thread = std::thread(&autonomous::AutonomousTask::navigateAll, this);
}

void AutonomousTask::navigateAll() {
	for (navtypes::point_t& point : _waypoint_coords_list) {
		_waypoint_coord = point;
		LOG_F(INFO, "*** Heading to new target: (%lf, %lf)", point[0], point[1]);
		json msg = {{"type", "auto_target_update"},
					{"latitude", point[0]},
					{"longitude", point[1]}};
		_server.sendJSON(Constants::MC_PROTOCOL_NAME, msg);
		navigate();
	}
}

void AutonomousTask::navigate() {
	commands::DriveToWaypointCommand cmd(_waypoint_coord, THETA_KP, DRIVE_VEL,
										 SLOW_DRIVE_THRESHOLD, DONE_THRESHOLD,
										 CLOSE_TO_TARGET_DUR_VAL);

	kinematics::DiffDriveKinematics diffDriveKinematics(Constants::EFF_WHEEL_BASE);

	auto sleepUntil = std::chrono::steady_clock().now();
	while (!cmd.isDone()) {
		auto latestGPS = robot::readGPS();
		auto latestHeading = robot::readIMUHeading();

		if (latestGPS.isFresh(2000ms) && latestHeading.isFresh(2000ms)) {
			LOG_SCOPE_F(INFO, "AutoNav");
			auto gpsPosData = latestGPS.getData();
			navtypes::pose_t latestPos(gpsPosData.x(), gpsPosData.y(), latestHeading.getData());
			cmd.setState(latestPos, robot::types::dataclock::now());
			commands::command_t output = cmd.getOutput();
			auto scaledVels = diffDriveKinematics.ensureWithinWheelSpeedLimit(
				kinematics::DiffDriveKinematics::PreferredVelPreservation::PreferThetaVel,
				output.xVel, output.thetaVel, Constants::MAX_WHEEL_VEL);
			navtypes::point_t relTarget = util::toTransform(latestPos) * _waypoint_coord;
			robot::setCmdVel(scaledVels(2), scaledVels(0));

			// Logging current location
			auto gpsCoordData = robot::metersToGPS(gpsPosData);
			if (!gpsCoordData) {
				LOG_F(INFO, "	Coordinates are not getting logged.");
			} else {
				LOG_F(INFO, "	Current coordinates: (%lf, %lf)", gpsCoordData->lat, gpsCoordData->lon);
			}

			LOG_F(INFO, "		Heading velocity: %lf, ", scaledVels(2));
			LOG_F(INFO, "		Foward velocity: %lf", scaledVels(0));
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
