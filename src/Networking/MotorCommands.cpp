
#include "MotorCommands.h"
#include "../HindsightCAN/CANMotorUnit.h"

// See MotorCommands.h for documentation

std::map<int, std::string> motors {
    {LF_MOTOR, "front_left_motor"},
    {LB_MOTOR, "back_left_motor"},
    {RF_MOTOR, "front_right_motor"},
    {RB_MOTOR, "back_right_motor"}
};

void setMotor(int motor, double power) {
    if (motor > 3 || motor < 0 || power > 1 || power < -1) {
        return; // error
    }

    nlohmann::json message = {
        {"type", "motor"},
        {"motor", motors[motor]},
        {"mode", "PWM"},
        {"PWM target", power}
    };
    
    ParseMotorPacket(message);
}

void drive(double power) {
    driveAndTurn(power, 0);
}

void drive(double power, double heading) {
    double turn_speed = sqrt(abs(heading / 180));
    driveAndTurn(power, turn_speed);
}

void driveAndTurn(double power, double turn_speed) {
    nlohmann::json message = {
        {"type", "drive"},
        {"forward_backward", power},
        {"left_right", turn_speed}
    };
    
    ParseMotorPacket(message);
}

void turn(double turn_speed) {
    driveAndTurn(0, turn_speed);
}