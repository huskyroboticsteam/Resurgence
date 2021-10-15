#pragma once

struct command_t {
	double thetaVel;
	double xVel;
};

class CommandBase {
public:
	virtual command_t getOutput() = 0;
	virtual bool isDone() = 0;
};
