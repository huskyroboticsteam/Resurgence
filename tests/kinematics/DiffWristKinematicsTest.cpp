#include "../../src/kinematics/DiffWristKinematics.h"

#include <catch2/catch.hpp>

namespace {

void assertApprox(const gearpos_t& gp1, const gearpos_t& gp2, double eps = 1e-8) {
	REQUIRE(Approx(gp1.left).epsilon(eps) == gp2.left);
	REQUIRE(Approx(gp1.right).epsilon(eps) == gp2.right);
}

void assertApprox(const jointpos_t& jp1, const jointpos_t& jp2, double eps = 1e-8) {
	REQUIRE(Approx(jp1.pitch).epsilon(eps) == jp2.pitch);
	REQUIRE(Approx(jp1.roll).epsilon(eps) == jp2.roll);
}

void assertInBounds(const gearpos_t& gpwr) {
	REQUIRE((gpwr.left >= -1 && gpwr.left <= 1));
	REQUIRE((gpwr.right >= -1 && gpwr.right <= 1));
}

void assertInBounds(const jointpos_t& jpwr) {
	REQUIRE((jpwr.pitch >= -1 && jpwr.pitch <= 1));
	REQUIRE((jpwr.roll >= -1 && jpwr.roll <= 1));
}

constexpr float PI = M_PI;
constexpr float degToRad(float deg) {
	return PI * deg / 180;
}

} // namespace

TEST_CASE("Position values are correct", "[kinematics]") {
	DiffWristKinematics kinematics;

	// test positions where only one axis is rotated
	for (int pitch = -90; pitch <= 90; pitch += 10) {
		float pitch_rad = degToRad(pitch);
		gearpos_t desired_gear_pos{pitch_rad, pitch_rad};
		jointpos_t desired_joint_pos{pitch_rad, 0};

		assertApprox(desired_gear_pos, kinematics.jointPosToGearPos(desired_joint_pos));
		assertApprox(desired_joint_pos, kinematics.gearPosToJointPos(desired_gear_pos));
	}

	for (int roll = -90; roll <= 90; roll += 10) {
		float roll_rad = degToRad(roll);
		gearpos_t desired_gear_pos{-roll_rad, roll_rad};
		jointpos_t desired_joint_pos{0, roll_rad};

		assertApprox(desired_gear_pos, kinematics.jointPosToGearPos(desired_joint_pos));
		assertApprox(desired_joint_pos, kinematics.gearPosToJointPos(desired_gear_pos));
	}

	// test a few known positions where both axes are rotated
	assertApprox(jointpos_t{degToRad(90), degToRad(90)},
				 kinematics.gearPosToJointPos(gearpos_t{0, degToRad(180)}));
	assertApprox(jointpos_t{degToRad(-90), degToRad(90)},
				 kinematics.gearPosToJointPos(gearpos_t{degToRad(-180), 0}));
}

TEST_CASE("Power values are normalized appropriately", "[kinematics]") {
	DiffWristKinematics kinematics;

	constexpr int num_steps = 20;
	constexpr float scale = 2.0f / num_steps;
	// take some discrete values across the entire space of pitch, roll power
	for (int p = 0; p <= num_steps; p++) {
		for (int r = 0; r <= num_steps; r++) {
			float pitch = (p * scale) - 1.0f;
			float roll = (r * scale) - 1.0f;
			jointpos_t desired_joint_power{pitch, roll};
			assertInBounds(desired_joint_power);

			gearpos_t gear_power = kinematics.jointPowerToGearPower(desired_joint_power);
			// the desired joint power may have required a gear power that is out of bounds, in
			// which case the gear power should be scaled down to fit the [-1,1] range. When
			// converting it back to joint power, the ratio between the two components should
			// be the same between the scaled joint power and the desired joint power.
			assertInBounds(gear_power);
			jointpos_t scaled_joint_power = kinematics.gearPowerToJointPower(gear_power);
			assertInBounds(scaled_joint_power);
			// instead of checking a/b == c/d, check ad == bc to avoid division by zero
			REQUIRE(Approx(desired_joint_power.pitch * scaled_joint_power.roll) ==
					scaled_joint_power.pitch * desired_joint_power.roll);
		}
	}

}
