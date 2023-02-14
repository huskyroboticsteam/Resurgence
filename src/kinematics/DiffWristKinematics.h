#pragma once

#include <Eigen/Dense>

namespace kinematics { 
/**
   \brief Represents the positions (or power) of the two motor-driven gears in the differential
   wrist.

   Contains position/power values for the left and right gears; when looking at the
   differential down the pitch axis, with the end effector to the right, the positive direction
   (for both gears) is counter-clockwise.
 */
struct gearpos_t {
	/**
	   \brief The position (or power) of the left gear.

	   When looking at the differential down the pitch axis, with the end effector to the
	   right, the positive direction is counter-clockwise, and the negative direction is
	   clockwise.
	 */
	float left;
	/**
	   \brief The position (or power) of the right gear.

	   When looking at the differential down the pitch axis, with the end effector to the
	   right, the positive direction is counter-clockwise, and the negative direction is
	   clockwise.
	 */
	float right;
	explicit gearpos_t(float left, float right);
	/**
	   \brief Constructs a set of gear positions from a 2-component vector as \f$\langle left,
	   right\rangle \f$.
	 */
	gearpos_t(const Eigen::Vector2f& vec);
	/**
	   \brief Converts a set of gear positions to a 2-component vector as \f$\langle left,
	   right\rangle \f$.
	 */
	Eigen::Vector2f vec() const;
};

/**
   \brief Represents the position (or power) of the differential wrist joint.

   Contains pitch/roll values for the end effector.
 */
struct jointpos_t {
	/**
	   \brief The pitch of the end effector.

	   When looking down the pitch axis, with the end effector to the right, the positive
	   direction is counter-clockwise, and the negative direction is clockwise.
	 */
	float pitch;
	/**
	   \brief The roll of the end effector.

	   When looking down the roll axis, with the end effector in front, the positive direction
	   is counter-clockwise, and the negative direction is clockwise.
	 */
	float roll;
	explicit jointpos_t(float pitch, float roll);
	/**
	   \brief Constructs a joint position from a 2-component vector as \f$\langle pitch,
	   roll\rangle \f$.
	 */
	jointpos_t(const Eigen::Vector2f& vec);
	/**
	   \brief Converts a set of gear positions to a 2-component vector as \f$\langle pitch,
	   roll\rangle \f$.
	 */
	Eigen::Vector2f vec() const;
};

/**
   \brief Represents the kinematics of a differential wrist

   A differential wrist is capable of adjusting both the pitch and roll of an end effector by
   driving two gears, one on each side. This class can be used to calculate the positions or
   motor powers of the gears given a desired joint position/power, and vice versa.
 */
class DiffWristKinematics {
public:
	/**
	   \brief Given a set of gear positions, calculate the resulting position of the joint.
	 */
	jointpos_t gearPosToJointPos(const gearpos_t& gearPos) const;
	/**
	   \brief Given a joint position, calculate the positions of the two gears.
	*/
	gearpos_t jointPosToGearPos(const jointpos_t& jointPos) const;
	/**
	 * \brief Given the motor power of each gear, calculate the resulting power of each axis of
	 * the joint.
     *
	 * \param gearPwr The motor power of each gear; each value in the structure should be in
	 * the range \f$[-1, 1]\f$.
	 * \return The resulting power of each axis, in the range \f$[-1, 1]\f$.
     *
	 * \warning This method will always return a normalized power value in the range \f$[-1,
	 * 1]\f$; if the given gear power values would cause the joint power to be outside of this
	 * range (which should only happen if the gear power values themselves were outside of this
	 * range) then the resulting vector will be scaled to be in this range, preserving the
	 * ratio between components.
	 */
	jointpos_t gearPowerToJointPower(const gearpos_t& gearPwr) const;
	/**
	 * \brief Given the desired power of each axis of the joint, calculate the required motor
	 * power for each of the gears.
     *
	 * \param jointPwr The desired power of each axis of the joint; each value in the structure
	 * should be in the range \f$[-1, 1]\f$.
	 * \return The required power of each of the gears, in the range \f$[-1, 1]\f$.
     *
	 * \warning Not all combinations of joint power are possible! This method will always
	 * return normalized gear power values in the range \f$[-1, 1]\f$; if the given joint power
	 * values would cause any gear power value to be outside of this range, then the resulting
	 * vector will be scaled to be in this range, preserving the ratio between components.
	 */
	gearpos_t jointPowerToGearPower(const jointpos_t& jointPwr) const;
};


}
