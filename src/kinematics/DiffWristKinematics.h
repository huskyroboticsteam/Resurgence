#pragma once

#include <Eigen/Dense>

struct gearpos_t {
	float lPos;
	float rPos;
	explicit gearpos_t(float lPos, float rPos);
	gearpos_t(const Eigen::Vector2f& vec);
	Eigen::Vector2f vec() const;
};

struct jointpos_t {
	float pitch;
	float roll;
	explicit jointpos_t(float pitch, float roll);
	jointpos_t(const Eigen::Vector2f& vec);
	Eigen::Vector2f vec() const;
};

class DiffWristKinematics {
public:
	jointpos_t gearPosToJointPos(float lPos, float rPos) const;
	jointpos_t gearPosToJointPos(const gearpos_t& gearPos) const;
	gearpos_t jointPosToGearPos(float pitch, float roll) const;
	gearpos_t jointPosToGearPos(const jointpos_t& jointPos) const;
};
