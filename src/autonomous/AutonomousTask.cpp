#include "AutonomousTask.h"

#include "../Constants.h"
#include "../commands/DriveToWaypointCommand.h"
#include "../control_interface.h"
#include "../utils/transform.h"
#include "../world_interface/world_interface.h"

#include <loguru.hpp>

#include <iostream>
#include <fstream>	
#include <string>

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

void AutonomousTask::start(const navtypes::points_t& waypointCoords, const bool circleMode,
						   const std::optional<double> radius, const std::optional<TaskType> type) {
	if (_autonomous_task_thread.joinable()) {
		kill();
	}

	if (_debug) {
		_logFile.open("log.csv", std::ios::out | std::ios::app);
		LOG_F(INFO, "file is open %d\n", _logFile.is_open());
	}
	_kill_called = false;
	
	if (circleMode) {
		if (type) {
			switch (type.value()) {
				case TaskType::TAG1:
					_autonomous_task_thread = std::thread(&autonomous::AutonomousTask::circleNavigation, this, waypointCoords[0], 7.5);
				case TaskType::TAG2:
					_autonomous_task_thread = std::thread(&autonomous::AutonomousTask::circleNavigation, this, waypointCoords[0], 15);
			}
		} else if (radius) {
			_autonomous_task_thread = std::thread(&autonomous::AutonomousTask::circleNavigation, this, waypointCoords[0], *radius);
		} else { // if no circle type or radius specified, default to radius 10
			_autonomous_task_thread = std::thread(&autonomous::AutonomousTask::circleNavigation, this, waypointCoords[0], 10);
		}
	} else {
		_waypoint_coords_list = waypointCoords;
		_autonomous_task_thread = std::thread(&autonomous::AutonomousTask::navigateAll, this);
	}
}

void AutonomousTask::circleNavigation(const navtypes::point_t& center, const double radius) {
	LOG_SCOPE_F(INFO, "AutoNav:Circle");
	double distanceBetweenPoints = radius / 10; // proportionally assign distance between points
	int numPoints = std::max(1, (int)round(2 * M_PI * radius / distanceBetweenPoints));
	double angleIncrement = 2 * M_PI / numPoints;												
	navtypes::points_t circlePoints;
 
	for (int i = 0; i <= numPoints; i++) {
		double angle = i * angleIncrement;
		double x = center[0] + radius * cos(angle);
		double y = center[1] + radius * sin(angle);
		circlePoints.push_back({x, y, 1});
	}
	_waypoint_coords_list = circlePoints;

	while (!_target_found) {
		navigateAll();
	}
}

void AutonomousTask::navigateAll() {
	LOG_SCOPE_F(INFO, "AutoNav:List");
	for (navtypes::point_t& point : _waypoint_coords_list) {
		_waypoint_coord = point;
		auto gpsCoord = robot::metersToGPS(point);
		if(!gpsCoord) {
			LOG_F(WARNING, "No GPS converter initialized!");
			return;
		}

		if (_debug) LOG_F(INFO, "*** Heading to new target: (%lf, %lf)", point[0], point[1]);

		json msg = {{"type", "auto_target_update"},
					{"latitude", gpsCoord->lat},
					{"longitude", gpsCoord->lon}};
		_server.sendJSON(Constants::MC_PROTOCOL_NAME, msg);
		navigate();

		if (_target_found) return;
	}

	if (_debug) _logFile.close();
}

void AutonomousTask::navigate() {
	commands::DriveToWaypointCommand cmd(_waypoint_coord, THETA_KP, DRIVE_VEL, DONE_THRESHOLD);

	kinematics::DiffDriveKinematics diffDriveKinematics(Constants::EFF_WHEEL_BASE);
	auto start = std::chrono::steady_clock::now();
	auto sleepUntil = std::chrono::steady_clock().now();
	while (!cmd.isDone()) {
		if (_target_found) {
			return;
		}
		auto latestGPS = robot::readGPS();
		auto latestHeading = robot::readIMUHeading();

		if (latestGPS.isFresh(2000ms) && latestHeading.isFresh(2000ms)) {
			auto gpsPosData = latestGPS.getData();
			navtypes::pose_t latestPos(gpsPosData.x(), gpsPosData.y(), latestHeading.getData());
			cmd.setState(latestPos);
			commands::command_t output = cmd.getOutput();
			auto scaledVels = diffDriveKinematics.ensureWithinWheelSpeedLimit(
				kinematics::DiffDriveKinematics::PreferredVelPreservation::PreferThetaVel,
				output.xVel, output.thetaVel, Constants::MAX_WHEEL_VEL);
			navtypes::point_t relTarget = util::toTransform(latestPos) * _waypoint_coord;
			robot::setCmdVel(scaledVels(2), scaledVels(0));
			

			if (_debug) {
				_logFile << gpsPosData.x() << "," << gpsPosData.y() << std::endl;
				_logFile.flush();

				navtypes::pose_t latestPos(gpsPosData.x(), gpsPosData.y(), 0.0);

				auto gpsCoord = robot::metersToGPS(gpsPosData);
				auto waypointGPSCoord = robot::metersToGPS(_waypoint_coord);
				double dist = (latestPos.topRows<2>() - _waypoint_coord.topRows<2>()).norm();

				LOG_F(INFO, "		Heading velocity: %lf ", scaledVels(2));
				LOG_F(INFO, "		Foward velocity: %lf", scaledVels(0));
				LOG_F(INFO, "		Distance: %lf", dist);
			}			
		}

		std::unique_lock autonomousTaskLock(_autonomous_task_mutex);
		sleepUntil += 20ms;

		// Wait 20ms or return if kill called
		if (_autonomous_task_cv.wait_until(autonomousTaskLock, sleepUntil,
										   [&] { return _kill_called; })) {
			return;
		}
	}
	auto end = std::chrono::steady_clock().now();
	std::chrono::duration<double> elapsed_seconds = end - start;
	
	if (_debug) {
		auto latestGPS = robot::readGPS();
		auto gpsPosData = latestGPS.getData();
		navtypes::pose_t latestPos(gpsPosData.x(), gpsPosData.y(), 0.0);

		auto gpsCoord = robot::metersToGPS(gpsPosData);
		auto waypointGPSCoord = robot::metersToGPS(_waypoint_coord);
		double dist = (latestPos.topRows<2>() - _waypoint_coord.topRows<2>()).norm();
		LOG_F(INFO, "*** Reached target waypoint! ***");
		LOG_F(INFO, "Distance to target on arrival: %lf", dist);
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
