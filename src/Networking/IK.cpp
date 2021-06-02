
#include "IK.h"
#include "ParseBaseStation.h"
#include "motor_interface.h"
#include "../Constants.h"
#include "../Globals.h"
#include "../log.h"
#include <iostream>

// Angular positions from the motor boards are given in millidegrees
double RAD_PER_INT = (2*M_PI) / (360*1000.0);

// See IK.h for explanation of sign / offset conventions
//                                     arm_base, shoulder,                elbow
const std::array<int, 3> sign_flips = {1,        1,                       1};
const std::array<double, 3> offsets = {0.0,      Constants::SHOULDER_MAX, Constants::ELBOW_MAX};

int32_t radToInt(double d, int motor_serial)
{
  double offset = offsets[motor_serial-1];
  int sign_flip = sign_flips[motor_serial-1];
  int32_t i = sign_flip * (d - offset) / RAD_PER_INT;
  log(LOG_TRACE, "radToInt %f %d %f %d %d\n", d, motor_serial, offset, sign_flip, i);
  return i;
}

double intToRad(int32_t i, int motor_serial)
{
  double offset = offsets[motor_serial-1];
  int sign_flip = sign_flips[motor_serial-1];
  double d = sign_flip * i * RAD_PER_INT + offset;
  log(LOG_TRACE, "intToRad %d %d %f %d %f\n", i, motor_serial, offset, sign_flip, d);
  return d;
}

std::array<double, 3> forward_kinematics() {
  // This function assumes we already received encoder data from each motor
  int arm_base_pos = Globals::status_data["arm_base"]["angular_position"];
  int shoulder_pos = Globals::status_data["shoulder"]["angular_position"];
  int elbow_pos = Globals::status_data["elbow"]["angular_position"];
  // See IK.h for explanation of sign / offset conventions
  double arm_base = intToRad(arm_base_pos, 1);
  double shoulder = intToRad(shoulder_pos, 2);
  double elbow = intToRad(elbow_pos, 3);
  log(LOG_DEBUG, "Running forward kinematics for %f %f %f\n", arm_base, shoulder, elbow);
  double r1 = Constants::SHOULDER_LENGTH * cos(shoulder);
  double r2 = Constants::ELBOW_LENGTH * cos(shoulder-elbow);
  double z1 = Constants::SHOULDER_LENGTH * sin(shoulder);
  double z2 = Constants::ELBOW_LENGTH * sin(shoulder-elbow);
  double z = z1+z2;
  double r = r1+r2;
  double x = r * cos(arm_base);
  double y = r * sin(arm_base);
  return {x,y,z};
}

bool ParseIKPacket(json &message) {
  const double ELBOW_LENGTH = Constants::ELBOW_LENGTH;
  const double SHOULDER_LENGTH = Constants::SHOULDER_LENGTH;
  if (message["wrist_base_target"] != nullptr)
  {
    json target = message["wrist_base_target"];
    double x, y, z;
    try {
      x = target[0];
      y = target[1];
      z = target[2];
    }
    catch (json::type_error)
    {
      return sendError("Could not parse wrist_base_target");
    }
    double baseAngle = atan2(y, x);
    double forward = sqrt(x*x + y*y);
    double height = z;
    // TODO right now we require the base angle to be within 90 degrees of forward.
    // This avoids some gimbal lock issues but limits the targets we can reach.
    if (baseAngle > M_PI/2)
    {
      baseAngle -= M_PI;
      forward *= -1.0;
    } else if (baseAngle < -M_PI/2) {
      baseAngle += M_PI;
      forward *= -1.0;
    }

    if (baseAngle > Constants::ARM_BASE_MAX || baseAngle < Constants::ARM_BASE_MIN) {
      return sendError("IK solution outside joint limits for arm base");
    }

    double crossSection = sqrt(height*height + forward*forward);
    double shoulderAngleA = atan2(height, forward);
    double cosElbowAngle = (crossSection*crossSection
      - ELBOW_LENGTH*ELBOW_LENGTH - SHOULDER_LENGTH*SHOULDER_LENGTH)
      /(-2*SHOULDER_LENGTH*ELBOW_LENGTH);
    if (abs(cosElbowAngle) >= 1.0)
    {
      return sendError("Infeasible IK target");
    }
    double elbowInsideAngle = acos(cosElbowAngle);
    double shoulderAngleB = asin(sin(elbowInsideAngle)*ELBOW_LENGTH/crossSection);
    double shoulderAngle = shoulderAngleA + shoulderAngleB;
    double elbowAngle = M_PI - elbowInsideAngle;
    if (shoulderAngle > Constants::SHOULDER_MAX || shoulderAngle < Constants::SHOULDER_MIN) {
      return sendError("IK solution outside joint limits for shoulder");
    }
    if (elbowAngle > Constants::ELBOW_MAX || elbowAngle < Constants::ELBOW_MIN) {
      return sendError("IK solution outside joint limits for elbow");
    }

    // Don't send these packets until we're sure the IK problem is feasible
    json base_packet = {{"type", "motor"},
                        {"motor", "arm_base"},
                        {"PID target", radToInt(baseAngle, 1)}}; // TODO offset, flip
    if (!ParseMotorPacket(base_packet))
    {
      return false;
    }
    json shoulder_packet = {{"type", "motor"},
                            {"motor", "shoulder"},
                            {"PID target", radToInt(shoulderAngle, 2)}};
    if (!ParseMotorPacket(shoulder_packet))
    {
      return false;
    }
    json elbow_packet = {{"type", "motor"},
                         {"motor", "elbow"},
                         {"PID target", radToInt(elbowAngle, 3)}};
    if (!ParseMotorPacket(elbow_packet))
    {
      return false;
    }
  }

  // TODO add functionality for "hand_orientation_target"
  return true;
}


