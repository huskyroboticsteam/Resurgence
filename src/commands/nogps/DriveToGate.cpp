#include "DriveToGate.h"
#include <iostream>

DriveToGate::DriveToGate(double radius, double thetaKP, double slowVel, double fastVel)
	: post({0, 0, 0}), radius(radius), thetaKP(thetaKP), slowVel(slowVel), fastVel(fastVel)
{
}

command_t DriveToGate::getOutput()
{
	if (post(2) != 1)
	{
		return {0, slowVel};
	}
	else if (!isDone())
	{
		double angle = atan2(post.y(), post.x());
		double vel = (post.topRows<2>().norm() <= 2 * radius) ? slowVel : fastVel;
		double targthvel = angle * thetaKP;
		double maxthvel = 0.5;
        std::cout << "Requesting vel " << vel << std::endl;
		if (abs(targthvel) > maxthvel) {
			targthvel = (targthvel > 0) ? maxthvel : -maxthvel;
		}
		return {targthvel, vel};
	}
	else
	{
		return {0, 0};
	}
}

bool DriveToGate::isDone()
{
	return post(2) == 1 && post.topRows<2>().norm() <= radius;
}

void DriveToGate::update(const point_t &p)
{
    if (p(2) != 0) this->post = p;
}
