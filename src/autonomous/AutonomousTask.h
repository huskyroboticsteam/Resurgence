#pragma once

#include "../navtypes.h"
#include "../network/websocket/WebSocketServer.h"


#include <cmath>
#include <condition_variable>
#include <mutex>
#include <thread>

#include <iostream>
#include <fstream>
#include <string>

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
	void start(const navtypes::points_t& waypointCoords);

	/**
	 * @brief Starts an autonomous task to circle around at a given radius around a given point.
	 *
	 * @param center the center of desired circle
	 * @param radius the radius of desired circle
	 */
	void circleAroundPoint(const navtypes::point_t& center, double radius);

	/**
	 * @brief Halts autonomous navigation by exiting the navigate() function and joining the
	 *        autonomous task thread
	 */
	void kill();

private:
	/**
	 * @brief Handles sequential navigation to a list of a waypoints, called by start()
	 */
	void navigateAll();

	/** 
	 * @brief Handles navigation to a single waypoint, called by navigateAll()
	 */
	void navigate();

	net::websocket::SingleClientWSServer& _server;

	navtypes::point_t _waypoint_coord; // current coord
	navtypes::points_t _waypoint_coords_list;
	
	std::mutex _autonomous_task_mutex;
	std::thread _autonomous_task_thread;
	std::condition_variable _autonomous_task_cv;
	bool _kill_called;

	std::ofstream _logFile;
};

} // namespace autonomous
