#include "DriveToGate.h"

DriveToGate::DriveToGate(double radius, double thetaKP, double slowVel, double fastVel)
	: post({0, 0, 0}), radius(radius), thetaKP(thetaKP), slowVel(slowVel), fastVel(fastVel)
{
}

command_t DriveToGate::getOutput()
{
	if (post(2) != 1)
	{
		return {0, slowVel};
	} else {
		double angle = atan2(post.y(), post.x());
		double vel = (post.topRows<2>().norm() <= 2 * radius) ? fastVel : slowVel;
		return {angle * thetaKP, vel};
	}
}

bool DriveToGate::isDone()
{
	return post(2) == 1 && post.topRows<2>().norm() <= radius;
}

void DriveToGate::update(const point_t &p)
{
	this->post = p;
}
