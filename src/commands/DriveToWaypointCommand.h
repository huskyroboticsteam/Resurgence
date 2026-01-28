#pragma once

#include "../navtypes.h"
#include "../utils/time.h"
#include "../world_interface/data.h"
#include "CommandBase.h"

#include <chrono>
#include <optional>

namespace commands {

/**
 * Provides robot drive velocity commands to navigate the robot to a given waypoint given
 * the velocity parameters to drive the robot at, the proportional factor to steer the robot
 * with and the 2D Cartesian position of the target in the world reference frame.
 */
class DriveToWaypointCommand : CommandBase {
public:
	/**
	 * Creates a new DriveToWaypointCommand with the specified targeting information.
	 *
	 * @param target the position of the target in world coordinates.
	 * @param thetaKP the scaling factor to use for robot steering proportional control.
	 * @param driveVel the speed to drive the robot at.
	 * @param doneThresh the threshold for minimum robot-target distance to complete command.
	 */
	DriveToWaypointCommand(const navtypes::point_t& target, double thetaKP, double driveVel,
						   double doneThresh);

	/**
	 * Must be called before getOutput() for each iteration. Updates the current pose of the
	 * robot in the global reference frame used by DriveToWaypointCommand.
	 *
	 * @param pose, the current pose of the robot in the global reference frame.
	 */
	void setState(const navtypes::pose_t& pose);

	/**
	 * Gets the angular velocity and forward speed to run the robot at in order to navigate
	 * robot to target.
	 *
	 * @return the angular velocity and forward speed to run the robot at in order to navigate
	 * robot to target.
	 */
	command_t getOutput() override;

	/**
	 * Returns whether the DriveToWaypointCommand has finished its navigation task.
	 *
	 * @return whether the DriveToWaypointCommand has finished its navigation task.
	 */
	bool isDone() override;

private:
	navtypes::point_t target;
	navtypes::pose_t pose;
	double thetaKP;
	double driveVel;	
	double doneThresh;	
	bool setStateCalledBeforeOutput;
};

} // namespace commands
