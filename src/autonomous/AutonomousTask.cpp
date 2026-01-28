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

void AutonomousTask::start(const navtypes::points_t& waypointCoords) {
	if (_autonomous_task_thread.joinable()) {
		kill();
	}

	if (_debug) {
		_logFile.open("log1.csv", std::ios::out | std::ios::app);
		LOG_F(INFO, "file is open %d\n", _logFile.is_open());
	}
	
	//_waypoint_coords_list = waypointCoords;
	// _waypoint_coords_list.push_back({0.0, 10.0, 1.0});
	// _waypoint_coords_list.push_back({10.0, 10.0, 1.0});
	_kill_called = false;
	AutonomousTask::circleAroundPoint(waypointCoords[0], 10);
	
	//_autonomous_task_thread = std::thread(&autonomous::AutonomousTask::navigateAll, this);
}

void AutonomousTask::circleAroundPoint(const navtypes::point_t& center, double radius) {
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
	_autonomous_task_thread = std::thread(&autonomous::AutonomousTask::navigateAll, this);
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
		LOG_F(INFO, "*** Heading to new target: (%lf, %lf)", point[0], point[1]);
		json msg = {{"type", "auto_target_update"},
					{"latitude", gpsCoord->lat},
					{"longitude", gpsCoord->lon}};
		_server.sendJSON(Constants::MC_PROTOCOL_NAME, msg);
		navigate();
	}
	_logFile.close();

}

void AutonomousTask::navigate() {
	commands::DriveToWaypointCommand cmd(_waypoint_coord, THETA_KP, DRIVE_VEL, DONE_THRESHOLD);

	kinematics::DiffDriveKinematics diffDriveKinematics(Constants::EFF_WHEEL_BASE);
	auto start = std::chrono::steady_clock::now();
	auto sleepUntil = std::chrono::steady_clock().now();
	while (!cmd.isDone()) {
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
			
		
			// Logging current and target location
			auto gpsCoord = robot::metersToGPS(gpsPosData);
			auto waypointGPSCoord = robot::metersToGPS(_waypoint_coord);
			double dist = (latestPos.topRows<2>() - _waypoint_coord.topRows<2>()).norm();
			// if (!gpsCoord || ! waypointGPSCoord) {
			// 	LOG_F(INFO, "	Coordinates are not getting logged.");
			// } else {
			//  LOG_F(INFO, "	Target: (%lf, %lf)", waypointGPSCoord->lat, waypointGPSCoord->lon);
			// 	LOG_F(INFO, "	Current coords: (%lf, %lf)", gpsPosData.x(), gpsPosData.y());
			// }
			
			// std::cout << "_logFile open? " << _logFile.is_open() << std::endl;
			LOG_F(INFO, "Heading: %lf", latestHeading.getData());
			_logFile << latestHeading.getData() << std::endl;
			//_logFile << gpsPosData.x() << "," << gpsPosData.y() << std::endl;
			_logFile.flush();

			// LOG_F(INFO, "		Heading velocity: %lf ", scaledVels(2));
			// LOG_F(INFO, "		Foward velocity: %lf", scaledVels(0));
			//sssLOG_F(INFO, "		Distance: %lf", dist);
		}

		std::unique_lock autonomousTaskLock(_autonomous_task_mutex);
		sleepUntil += 20ms;

		// Wait 500ms or return out of while loop if kill called
		if (_autonomous_task_cv.wait_until(autonomousTaskLock, sleepUntil,
										   [&] { return _kill_called; })) {
			return;
		}
	}
	auto latestGPS = robot::readGPS();
	auto gpsPosData = latestGPS.getData();
	navtypes::pose_t latestPos(gpsPosData.x(), gpsPosData.y(), 0.0);

	auto gpsCoord = robot::metersToGPS(gpsPosData);
	auto waypointGPSCoord = robot::metersToGPS(_waypoint_coord);
	double dist = (latestPos.topRows<2>() - _waypoint_coord.topRows<2>()).norm();

	auto end = std::chrono::steady_clock().now();
	std::chrono::duration<double> elapsed_seconds = end - start;
	// LOG_F(INFO, "*** Reached target waypoint! ***");
	LOG_F(INFO, "Distance to target on arrival: %lf", dist);
	//LOG_F(INFO, "Time taken to reach waypoint: %lf seconds", elapsed_seconds.count());
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
