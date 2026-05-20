#include "AutonomousTask.h"

#include "../Constants.h"
#include "../commands/PurePursuitCommand.h"
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
	if (_debug) _logFile.close();
	if (_autonomous_task_thread.joinable()) {
		_autonomous_task_thread.join();
	}
}

void AutonomousTask::start(const navtypes::points_t& waypointCoords, const bool circleMode,
						   const std::optional<double> radius, const std::optional<TaskType> type) {
	if (_autonomous_task_thread.joinable()) {
		kill();
	}

	if (waypointCoords.size() == 0) {
		return;
	}

	if (_debug) {
		_logFile.open("log.csv", std::ios::out | std::ios::app);
		LOG_F(INFO, "log file is open: %d\n", _logFile.is_open());
	}
	_kill_called = false;

	// for (auto& point : waypointCoords) {
	// 	_logFile << point[0] << "," << point[1] << std::endl;
	// }
	
	if (circleMode) {
		if (waypointCoords.size() > 1) {
			_waypoint_coords_list = 
					navtypes::points_t(waypointCoords.begin(), waypointCoords.end() - 1);
			_circle_center = waypointCoords.back();
			_autonomous_task_thread = 
					std::thread(&AutonomousTask::navigateThenCircle, this, radius, type);
		} else {
			_circle_center = waypointCoords[0];
			if (type) {
				switch (type.value()) {
					case TaskType::TAG1:
						_autonomous_task_thread = std::thread(
									&autonomous::AutonomousTask::circleNavigation, 
									this, 
									7.5,
									std::nullopt
								);
						break;
					case TaskType::TAG2:
						_autonomous_task_thread = std::thread(
									&autonomous::AutonomousTask::circleNavigation, 
									this,
									13,
									16							
								);
						break;
				}
			} else if (radius) {
				_autonomous_task_thread = std::thread(
							&autonomous::AutonomousTask::circleNavigation, 
							this,
							*radius,
							std::nullopt
						);
			} else { // if no circle type or radius specified, default to radius 10
				_autonomous_task_thread = std::thread(
							&autonomous::AutonomousTask::circleNavigation, 
							this, 
							10,
							std::nullopt
						);
			}
		}
	} else {
		_waypoint_coords_list = waypointCoords;
		_autonomous_task_thread = std::thread(&autonomous::AutonomousTask::navigateAll, this);
	}
}

void AutonomousTask::navigateThenCircle(const std::optional<double> radius,
						    const std::optional<Constants::autonomous::TaskType> type) {
	AutonomousTask::navigateAll();
	if (_kill_called) return;

	if (type) {
		switch (type.value()) {
			case TaskType::TAG1:
				AutonomousTask::circleNavigation(7.5, std::nullopt);
				break;
			case TaskType::TAG2:
				AutonomousTask::circleNavigation(13, 16);
				break;
		}
	} else if (radius) {
		AutonomousTask::circleNavigation(*radius, std::nullopt);
	} else {
		AutonomousTask::circleNavigation(10, std::nullopt);
	}
}

void AutonomousTask::circleNavigation(const double radius, const std::optional<double> radius2) {
	LOG_SCOPE_F(INFO, "AutoNav:Circle");
	if (radius == 0) {
		LOG_F(WARNING, "Unable to generate circle of radius 0. Enter a radius > 0!");
		return;
	}

	commands::PurePursuitCommand cmd1(generateCirclePoints(radius));
	auto cmd2 = radius2 ? std::optional<commands::PurePursuitCommand>(generateCirclePoints(*radius2)) : std::nullopt;

	while (!_target_found && !_kill_called) {
		cmd1.reset();
		navigate(cmd1);
		if (_target_found || _kill_called) break;

		if (cmd2) {
			cmd2->reset();
			navigate(*cmd2);
		}
	}
}

navtypes::points_t AutonomousTask::generateCirclePoints(const double radius) {
	double distanceBetweenPoints = radius / 10; // proportionally assign distance between points
	int numPoints = std::max(1, (int)round(2 * M_PI * radius / distanceBetweenPoints));
	double angleIncrement = 2 * M_PI / numPoints;												
	navtypes::points_t circlePoints;
 
	auto latestGPS = robot::readGPS();
	auto gpsPosData = latestGPS.getData();
	// _logFile << gpsPosData.x() << "," << gpsPosData.y() << std::endl;
	double startAngle = std::atan2(gpsPosData.y() - _circle_center[1],
								   gpsPosData.x() - _circle_center[0]);

								//  ***  round up to nearest multiple of angleIncrement  ***

	LOG_F(INFO, "start angle: %f", startAngle);
	for (int i = 0; i <= numPoints; i++) {
		double angle = startAngle + i * angleIncrement;
		double x = _circle_center[0] + radius * cos(angle);
		double y = _circle_center[1] + radius * sin(angle);
		circlePoints.push_back({x, y, 1});
		_logFile << x << "," << y << std::endl;
	}
	return circlePoints;
}

void AutonomousTask::navigateAll() {
	LOG_SCOPE_F(INFO, "AutoNav:List");
	commands::PurePursuitCommand cmd(_waypoint_coords_list);
	navigate(cmd);
}

void AutonomousTask::navigate(commands::PurePursuitCommand& cmd) {

	kinematics::DiffDriveKinematics diffDriveKinematics(Constants::EFF_WHEEL_BASE);
	auto sleepUntil = std::chrono::steady_clock().now();

	while (!cmd.isDone()) {
		/*
		 * planned pseudo-code
		 * if (!_target_found && target pinpointed) {
		 * 	   _target_found = true;
		 *     _waypoint_coords = {} // necessary?
		 * 	   _waypoint_coord = new coord
		 * 	   return; 
		 * }
		 */
		auto latestGPS = robot::readGPS();
		auto latestHeading = robot::readIMUHeading();

		if (latestGPS.isFresh(2000ms) && latestHeading.isFresh(2000ms)) {
			auto now = std::chrono::steady_clock::now();
			
			auto gpsPosData = latestGPS.getData();
			navtypes::pose_t latestPos(gpsPosData.x(), gpsPosData.y(), latestHeading.getData());
			cmd.setState(latestPos);
			commands::command_t output = cmd.getOutput();
			auto scaledVels = diffDriveKinematics.ensureWithinWheelSpeedLimit(
				kinematics::DiffDriveKinematics::PreferredVelPreservation::PreferThetaVel,
				output.xVel, output.thetaVel, Constants::MAX_WHEEL_VEL);
			robot::setCmdVel(scaledVels(2), scaledVels(0));			

			// if (_debug) {
			// 	_logFile << gpsPosData.x() << "," << gpsPosData.y() << std::endl;
			// 	_logFile.flush();
			// }				
		}

		std::unique_lock autonomousTaskLock(_autonomous_task_mutex);
		sleepUntil += 20ms;

		// Wait 20ms or return if kill called
		if (_autonomous_task_cv.wait_until(autonomousTaskLock, sleepUntil,
										   [&] { return _kill_called; })) {
			return;
		}
	}

	// If navigation is done, send 0 velocity command.
	robot::setCmdVel(0.0, 0.0);
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