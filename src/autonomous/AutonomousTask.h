#pragma once

#include "../Constants.h"
#include "../navtypes.h"
#include "../network/websocket/WebSocketServer.h"


#include <cmath>
#include <condition_variable>
#include <mutex>
#include <thread>

#include <iostream>
#include <fstream>
#include <string>

using namespace Constants::autonomous;

namespace autonomous {

/*
 * @brief This class facilitates autonomous navigation to a specified waypoint.
 */
class AutonomousTask {
public:
	/**
	 * @brief Constructs a new AutonomousTask
	 */
	AutonomousTask(net::websocket::SingleClientWSServer&);

	/**
	 * @brief Destructs AutonomousTask object, joins _autonomous_task_thread
	 */
	~AutonomousTask();

	/**
	 * @brief Starts an autonomous task to the provided waypoints
	 *
	 * @param waypointCoords the list of waypoints to navigate to
	 */
	void start(const navtypes::points_t& waypointCoords, const bool circleMode,
			   const std::optional<double> radius, const std::optional<TaskType> type);


	/**
	 * @brief Halts autonomous navigation by exiting the navigate() function and joining the
	 *        autonomous task thread
	 */
	void kill();

private:
	/**
	 * @brief Starts an autonomous task to circle around at a given radius around a given point.
	 *
	 * @param center the center of desired circle
	 * @param radius the radius of desired circle
	 */
	void circleNavigation(const navtypes::point_t& center, const double radius, const std::optional<double> radius2);

	/**
	 * @brief Handles sequential navigation to a list of a waypoints, called by start()
	 */
	void navigateAll();

	/** 
	 * @brief Handles navigation to a single waypoint
	 */
	void navigate();

	/**
	 * @brief Helper function for circleNavigation().
	 * 		  Generates a vector of points that approximates a circle of given radius and center.
	 * @param center the center of desired circle
	 * @param radius the radius of desired circle
	 * @return points_t filled with evenly spaced out points on circle.
	 */
	navtypes::points_t generateCirclePoints(const navtypes::point_t& center, const double radius);

	net::websocket::SingleClientWSServer& _server;

	navtypes::point_t _waypoint_coord; // current coord
	navtypes::points_t _waypoint_coords_list;
	bool _target_found = false;

	std::mutex _autonomous_task_mutex;
	std::thread _autonomous_task_thread;
	std::condition_variable _autonomous_task_cv;
	bool _kill_called;

	bool _debug = true; // toggle debugging logs
	/*
	 *  if _debug is true, a _logFile called "log.csv" is created in the build directory
	 *	and populated with the actual rover path.
	 */
	std::ofstream _logFile;
};

} // namespace autonomous
