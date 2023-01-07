#include "../navtypes.h"
#include "../world_interface/data.h"
#include "CommandBase.h"

#include <chrono>
#include <optional>

namespace commands {

class DriveToWaypointCommand : CommandBase {
public:
	DriveToWaypointCommand(const navtypes::point_t& target, double thetaKP, double driveVel,
						   double slowDriveVel, double doneThresh);
	void setState(const navtypes::pose_t& pose);
	command_t getOutput() override;
	bool isDone() override;

private:
	navtypes::point_t target;
	navtypes::pose_t pose;
	double thetaKP;
	double driveVel;
	double slowDriveVel;
	double doneThresh;
	std::optional<robot::types::datatime_t> closeToTargetStartTime;
};

} // namespace commands
