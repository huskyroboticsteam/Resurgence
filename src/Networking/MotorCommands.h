
#ifndef MOTOR_COMMANDS_H
#define MOTOR_COMMANDS_H

#include "CANUtils.h"
#include "json.hpp"
#include "ParseBaseStation.h"

#define LF_MOTOR 0
#define RF_MOTOR 1
#define LB_MOTOR 2
#define RB_MOTOR 3

// motor is the desired motor id (defined above)
// power should be [-1,1] (positive is forwards, negative is backwards)
// (NOTE: not simply counter-clockwise/clockwise)
void setMotor(int motor, double power);

// power should be [-1,1] (positive is forwards, negative is backwards)
void drive(double power);

// turn_speed should be [-1,1] (positive is counter-clockwise, negative is clockwise)
void turn(double turn_speed);

// power should be [-1,1] (positive is forwards, negative is backwards)
// turn_speed should be [-1,1] (positive is counter-clockwise, negative is clockwise)
// motor powers get normalized to 1 at the end if they're too high
void driveAndTurn(double power, double turn_speed);

// power should be [-1,1] (positive is forwards, negative is backwards)
// heading should be [-180, 180] (0 is forward, positive is left)
void drive(double power, double heading);

#endif
