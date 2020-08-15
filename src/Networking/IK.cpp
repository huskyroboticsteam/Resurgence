
#include "ParseBaseStation.h"
#include "motor_interface.h"
#include "../Constants.h"
#include <iostream>

// TODO check with electronics to see if this is accurate
double RAD_PER_INT = (2*M_PI) / ((double) (1<<16));

int32_t radToInt(double d, double offset, int32_t sign_flip)
{
  int32_t i = sign_flip * (d - offset) / RAD_PER_INT;
  return i;
}

double intToRad(int32_t i, double offset, int32_t sign_flip)
{
  double d = sign_flip * i * RAD_PER_INT + offset;
  return d;
}

bool ParseIKPacket(json &message) {
  double ELBOW_LENGTH = Constants::ELBOW_LENGTH;
  double SHOULDER_LENGTH = Constants::SHOULDER_LENGTH;
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
                        {"mode", "PID"},
                        {"PID target", radToInt(baseAngle, 0, 1)}}; // TODO offset, flip
    if (!ParseMotorPacket(base_packet))
    {
      return false;
    }
    json shoulder_packet = {{"type", "motor"},
                            {"motor", "shoulder"},
                            {"mode", "PID"},
                            {"PID target", radToInt(shoulderAngle, 0, 1)}};
    if (!ParseMotorPacket(shoulder_packet))
    {
      return false;
    }
    json elbow_packet = {{"type", "motor"},
                         {"motor", "elbow"},
                         {"mode", "PID"},
                         {"PID target", radToInt(elbowAngle, 0, 1)}};
    if (!ParseMotorPacket(elbow_packet))
    {
      return false;
    }
  }

  // TODO add functionality for "hand_orientation_target"
  return true;
}


