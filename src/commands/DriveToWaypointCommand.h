#pragma once

#include "../navtypes.h"
#include "../world_interface/data.h"
#include "CommandBase.h"

#include <chrono>
#include <optional>

namespace commands {

/**
 * Provides robot drive velocity commands to navigate the robot to a given waypoint given
 * the velocity parameters to drive the robot at, the proportional factor to steer the robot with
 * and the 2D Cartesian position of the target in the world reference frame.
 */
class DriveToWaypointCommand : CommandBase {
public:
	/**
	 * Creates a new DriveToWaypointCommand with the specified targeting information.
	 * 
	 * @param target, the position of the target in world coordinates.
	 * @param thetaKP, the scaling factor to use for robot steering proportional control.
	 * @param driveVel, the speed to drive the robot at.
	 * @param slowDriveVel, the slower speed to drive the robot at when near the target.
	 * @param doneThresh, the threshold for minimum robot-target distance to complete command.
	 * @param closeToTargetDurVal, the duration within doneThresh after which robot navigation 
	 * 							   is considered done, seconds.
	 */
	DriveToWaypointCommand(const navtypes::point_t& target, double thetaKP, double driveVel,
						   double slowDriveVel, double doneThresh, util::dseconds closeToTargetDur);

	/**
	 * Must be called before getOutput() for each iteration. Updates the current pose of the robot in 
	 * the global reference frame used by DriveToWaypointCommand.
	 * 
	 * @param pose, the current pose of the robot in the global reference frame.
	 * @param time, the current time from robot::types::dataclock.
	 */
	void setState(const navtypes::pose_t& pose, robot::types::datatime_t time);

	/**
	 * Gets the angular velocity and forward speed to run the robot at in order to navigate robot to target.
	 * 
	 * @return the angular velocity and forward speed to run the robot at in order to navigate robot to target.
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
	double slowDriveVel;
	double doneThresh;
	bool setStateCalledBeforeOutput;
	util::dseconds closeToTargetDur;
	std::optional<robot::types::datatime_t> lastRecordedTime;
	std::optional<robot::types::datatime_t> closeToTargetStartTime;
};

} // namespace commands
