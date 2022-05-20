#include "DiffWristKinematics.h"

#include <Eigen/Dense>

using Eigen::Matrix2f;
using Eigen::Vector2f;

gearpos_t::gearpos_t(const Vector2f& vec) : lPos(vec(0)), rPos(vec(1)) {}
gearpos_t::gearpos_t(float lPos, float rPos) : lPos(lPos), rPos(rPos) {}
Vector2f gearpos_t::vec() const {
	return Vector2f(lPos, rPos);
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

jointpos_t DiffWristKinematics::gearPosToJointPos(float lPos, float rPos) const {
	return this->gearPosToJointPos(gearpos_t{lPos, rPos});
}

jointpos_t DiffWristKinematics::gearPosToJointPos(const gearpos_t& gearPos) const {
	Vector2f res = gearToJointPosTransform * gearPos.vec();
	return res;
}

gearpos_t DiffWristKinematics::jointPosToGearPos(float pitch, float roll) const {
	return this->jointPosToGearPos(jointpos_t{pitch, roll});
}

gearpos_t DiffWristKinematics::jointPosToGearPos(const jointpos_t &jointPos) const {
	Vector2f res = jointToGearPosTransform * jointPos.vec();
	return res;
}
