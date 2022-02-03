#pragma once

#include "../kinematics/DiffDriveKinematics.h"

void setCmdVelToIntegrate(const wheelvel_t& wheelVels);

void setCmdVelToIntegrate(double dtheta, double dx);

