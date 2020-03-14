
#include "ParseBaseStation.h"
#include "../Constants.h"

// TODO check with electronics to see if this is accurate
double RAD_PER_INT = ((double) (2<<16)) / (2*M_PI);
int32_t PI_AS_INT = M_PI / RAD_PER_INT;

int32_t radToInt(double d, int32_t offset, int32_t sign_flip)
{
  int32_t i = sign_flip * d / RAD_PER_INT - offset;
  return i;
}

double intToRad(int32_t i, int32_t offset, int32_t sign_flip)
{
  double d = sign_flip * (i + offset) * RAD_PER_INT;
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
    double base_angle = atan2(y, x);
    json base_packet = {{"type", "motor"},
                        {"motor", "arm_base"},
                        {"mode", "PID"},
                        {"PID target", radToInt(base_angle, 0, 1)}}; // TODO offset, flip

    if (!ParseMotorPacket(base_packet))
    {
      return false;
    }

    double forward = sqrt(x*x + y*y);
    double height = z;

    double crossSection = sqrt(height*height + forward*forward);
    double shoulderAngleA = atan2(height, forward);
    double cosElbowAngle = (crossSection*crossSection
      - ELBOW_LENGTH*ELBOW_LENGTH - SHOULDER_LENGTH*SHOULDER_LENGTH)
      /(-2*SHOULDER_LENGTH*ELBOW_LENGTH);
    if (abs(cosElbowAngle) > 1.0)
    {
      return sendError("Infeasible IK target");
    }
    double elbowInsideAngle = acos(cosElbowAngle);
    double shoulderAngleB = asin(sin(elbowInsideAngle)*ELBOW_LENGTH/crossSection);
    double shoulderAngle = shoulderAngleA + shoulderAngleB;
    double elbowAngle = M_PI - elbowInsideAngle;

    json elbow_packet = {{"type", "motor"},
                         {"motor", "elbow"},
                         {"mode", "PID"},
                         {"PID target", radToInt(elbowAngle, PI_AS_INT, 1)}};
    json shoulder_packet = {{"type", "motor"},
                            {"motor", "shoulder"},
                            {"mode", "PID"},
                            {"PID target", radToInt(shoulderAngle, PI_AS_INT, 1)}};
    if (!ParseMotorPacket(elbow_packet))
    {
      return false;
    }
    if (!ParseMotorPacket(shoulder_packet))
    {
      return false;
    }
  }

  // TODO add functionality for "hand_orientation_target"
  return true;
}


