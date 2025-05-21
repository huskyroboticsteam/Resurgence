#pragma once

#include "../navtypes.h"

#include <cmath>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <queue>

namespace autonomous {

/*
 * @brief This class facilitates autonomous navigation to a specified waypoint.
 */
class AutonomousTask {
public:
	/*
	 * @brief Constructs a new AutonomousTask
	 */
	AutonomousTask();

	/*
	 * @brief Destructs AutonomousTask object, joins _autonomous_task_thread
	 */
	~AutonomousTask();

	/*
	 * @brief Starts an autonomous task to the provided waypoint
	 *
	 * @param navtypes::pose_t The waypoint to navigate to
	 */
	void start(std::queue<navtypes::point_t> waypoints);


	/*
	 * @brief Halts autonomous navigation by exiting the navigate() function and joining the
	 *        autonomous task thread
	 */
	void kill();

private:
	/*
	 * @brief Handles navigation to waypoint, called by start()
	 */
	std::queue<navtypes::point_t> _waypoint_queue;   
	void navigate();
	navtypes::point_t _waypoint_coords;
	std::mutex _autonomous_task_mutex;
	std::thread _autonomous_task_thread;
	std::condition_variable _autonomous_task_cv;
	bool _kill_called;
};

} // namespace autonomous
