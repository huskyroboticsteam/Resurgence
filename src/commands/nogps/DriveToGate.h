#pragma once

#include "../../simulator/utils.h"
#include "../CommandBase.h"

class DriveToGate : CommandBase {
public:
	DriveToGate(double radius, double thetaKP, double slowVel, double fastVel);
	void update(const point_t& p);
	command_t getOutput() override;
	bool isDone() override;

private:
	point_t post;
	double radius;
	double thetaKP;
	double slowVel;
	double fastVel;
};
