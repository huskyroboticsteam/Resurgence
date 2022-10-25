#include "DiffWristKinematics.h"

#include <Eigen/Dense>

using Eigen::Matrix2f;
using Eigen::Vector2f;

namespace kinematics {

gearpos_t::gearpos_t(const Vector2f& vec) : left(vec(0)), right(vec(1)) {}
gearpos_t::gearpos_t(float left, float right) : left(left), right(right) {}
Vector2f gearpos_t::vec() const {
	return Vector2f(left, right);
}

jointpos_t::jointpos_t(const Vector2f& vec) : pitch(vec(0)), roll(vec(1)) {}
jointpos_t::jointpos_t(float pitch, float roll) : pitch(pitch), roll(roll) {}
Vector2f jointpos_t::vec() const {
	return Vector2f(pitch, roll);
}

static Matrix2f createTransform() {
	Matrix2f ret;
	ret <<
		0.5, 0.5,
		-0.5, 0.5;
	return ret;
}

static const Matrix2f gearToJointPosTransform = createTransform();
static const Matrix2f jointToGearPosTransform = gearToJointPosTransform.inverse();


jointpos_t DiffWristKinematics::gearPosToJointPos(const gearpos_t& gearPos) const {
	Vector2f res = gearToJointPosTransform * gearPos.vec();
	return res;
}

gearpos_t DiffWristKinematics::jointPosToGearPos(const jointpos_t &jointPos) const {
	Vector2f res = jointToGearPosTransform * jointPos.vec();
	return res;
}

jointpos_t DiffWristKinematics::gearPowerToJointPower(const gearpos_t &gearPwr) const {
	Vector2f jointPwr = gearToJointPosTransform * gearPwr.vec();
	// compute infinity norm; i.e. component with max absolute value
	float maxVal = jointPwr.lpNorm<Eigen::Infinity>();
	// if any component is absolutely greater than 1, divide by maximum component to scale
	// everything to the interval [-1,1]
	if(maxVal > 1){
		jointPwr /= maxVal;
	}
	return jointPwr;
}

gearpos_t DiffWristKinematics::jointPowerToGearPower(const jointpos_t &jointPwr) const {
    Vector2f gearPwr = jointToGearPosTransform * jointPwr.vec();
	// compute infinity norm; i.e. component with max absolute value
	float maxVal = gearPwr.lpNorm<Eigen::Infinity>();
	// if any component is absolutely greater than 1, divide by maximum component to scale
	// everything to the interval [-1,1]
	if(maxVal > 1){
		gearPwr /= maxVal;
	}
	return gearPwr;
}
}
