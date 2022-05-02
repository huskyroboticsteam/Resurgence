#include "../Globals.h"
#include "../world_interface/world_interface.h"
#include "IK.h"
#include "NetworkingStubs.h"
#include "ParseBaseStation.h"
#include "ParseCAN.h"
#include "TestPackets.h"
#include "motor_interface.h"
extern "C" {
#include "../HindsightCAN/CANMotorUnit.h"
#include "../HindsightCAN/Port.h"
}
#include <iostream>

#include <catch2/catch.hpp>

using nlohmann::json;
using namespace robot;

void setupEncoders() {
	int32_t start_angle = 1337;
	Globals::status_data["arm_base"]["angular_position"] = start_angle;
	Globals::status_data["shoulder"]["angular_position"] = start_angle;
	Globals::status_data["elbow"]["angular_position"] = start_angle;
}

TEST_CASE("Basic CAN ID construction", "[CAN]") {
	uint16_t CAN_ID = ConstructCANID(PACKET_PRIORITY_NORMAL, DEVICE_GROUP_MOTOR_CONTROL, 0x05);
	uint16_t EXPECTED = 0x0505;
	REQUIRE(CAN_ID == EXPECTED);
}

TEST_CASE("Basic packet creation", "[CAN]") {
	uint16_t CAN_ID = ConstructCANID(PACKET_PRIORITY_NORMAL, DEVICE_GROUP_MOTOR_CONTROL, 0x05);
	uint8_t testDataPacket[2];
	testDataPacket[0] = 0x60;
	testDataPacket[1] = 0x43;
	CANPacket packet = ConstructCANPacket(CAN_ID, 0x02, testDataPacket);

	REQUIRE(packet.id == 0x0505);
	REQUIRE(packet.dlc == 0x02);
	REQUIRE(packet.data[0] == 0x60);
	REQUIRE(packet.data[1] == 0x43);
}

TEST_CASE("ParseCAN can handle telemetry", "[CAN]") {
	ParseCANPacket(motorTelemetry());
	std::cout << Globals::status_data << std::endl;
	REQUIRE(Globals::status_data.dump() == "{\"front_right_wheel\":{\"current\":256}}");
}

TEST_CASE("Can handle malformed packets", "[BaseStation]") {
	char const* msg = "{\"type\": \"moto";
	REQUIRE(ParseBaseStationPacket(msg) == false);
	json m = json::parse(popBaseStationPacket());
	REQUIRE(m["msg"] == "Parse error");
}

TEST_CASE("Ignores invalid motor names", "[BaseStation]") {
	char const* msg = "{\"type\": \"motor\", \"motor\": \"xyzzy\", \"mode\": \"PID\"}";
	REQUIRE(ParseBaseStationPacket(msg) == false);
	json m = json::parse(popBaseStationPacket());
	REQUIRE(m["msg"] == "Unrecognized motor xyzzy");
}

TEST_CASE("Does not allow incremental PID without encoder data", "[BaseStation]") {
	clearTestGlobals();
	char const* msg =
		"{\"type\": \"motor\", \"motor\": \"arm_base\", \"incremental PID speed\":1.0}";
	REQUIRE(ParseBaseStationPacket(msg) == false);
	json m = json::parse(popBaseStationPacket());
	REQUIRE(m["msg"] == "Cannot use PID for motor arm_base: No encoder data yet");
	REQUIRE(numCANPackets() == 0);
}

TEST_CASE("Allows IK axis as 'motor' when using IK", "[BaseStation]") {
	clearTestGlobals();
	setupEncoders();
	char const* msg =
		"{\"type\": \"motor\", \"motor\": \"IK_X\", \"incremental IK speed\":1.0}";
	REQUIRE(ParseBaseStationPacket(msg) == false);
	REQUIRE(Globals::status_data["arm_base"]["operation_mode"] == "incremental IK speed");
	REQUIRE(Globals::status_data["shoulder"]["operation_mode"] == "incremental IK speed");
	REQUIRE(Globals::status_data["elbow"]["operation_mode"] == "incremental IK speed");
	json m = json::parse(popBaseStationPacket());
	REQUIRE(m["msg"] == "IK is not yet supported");
	// TODO once implemented, make sure IK incremental speed values are set properly
	// REQUIRE(Globals::status_data["IK_X"]["???"] == "???");
}

TEST_CASE("Does not allow IK axis as 'motor' for non-IK modes", "[BaseStation]") {
	clearTestGlobals();
	char const* msg =
		"{\"type\": \"motor\", \"motor\": \"IK_X\", \"incremental PID speed\":1.0}";
	REQUIRE(ParseBaseStationPacket(msg) == false);
	json m = json::parse(popBaseStationPacket());
	REQUIRE(m["msg"] == "Invalid operation mode 'incremental PID speed' for motor IK_X");
}

TEST_CASE("Incremental PID", "[BaseStation]") {
	clearTestGlobals();
	int32_t start_angle = 1337;
	Globals::status_data["arm_base"]["angular_position"] = start_angle;
	CANPacket p;

	char const* msg =
		"{\"type\": \"motor\", \"motor\": \"arm_base\", \"incremental PID speed\":1.0}";
	REQUIRE(ParseBaseStationPacket(msg) == true);
	REQUIRE(numCANPackets() == 4); // mode set + 3 PID coefficients
	p = popCANPacket();
	REQUIRE(GetDeviceSerialNumber(&p) == 1);
	REQUIRE(PacketIsOfID(&p, ID_MOTOR_UNIT_MODE_SEL));
	REQUIRE(p.data[1] == MOTOR_UNIT_MODE_PID);
	p = popCANPacket();
	REQUIRE(PacketIsOfID(&p, ID_MOTOR_UNIT_PID_P_SET));
	p = popCANPacket();
	REQUIRE(PacketIsOfID(&p, ID_MOTOR_UNIT_PID_I_SET));
	p = popCANPacket();
	REQUIRE(PacketIsOfID(&p, ID_MOTOR_UNIT_PID_D_SET));

	REQUIRE(Globals::status_data["arm_base"]["operation_mode"] == "incremental PID speed");
	REQUIRE(Globals::status_data["arm_base"]["target_angle"] == start_angle);
	int expected_increment = 360 * 1000 * ((M_PI / 8) / (2 * M_PI)) / 10;
	REQUIRE(Globals::status_data["arm_base"]["millideg_per_control_loop"] ==
			expected_increment);
	REQUIRE(Globals::status_data["arm_base"]["most_recent_command"] > 0);

	msg = "{\"type\": \"motor\", \"motor\": \"arm_base\", \"incremental PID speed\":-0.5}";
	REQUIRE(ParseBaseStationPacket(msg) == true);
	REQUIRE(numCANPackets() == 0); // Shouldn't change the mode a second time
	expected_increment *= -0.5;
	REQUIRE(Globals::status_data["arm_base"]["millideg_per_control_loop"] ==
			expected_increment);

	incrementArm();
	REQUIRE(numCANPackets() == 1);
	p = popCANPacket();
	REQUIRE(GetDeviceSerialNumber(&p) == 1);
	REQUIRE(PacketIsOfID(&p, ID_MOTOR_UNIT_PID_POS_TGT_SET));
	REQUIRE(GetPIDTargetFromPacket(&p) == start_angle + expected_increment);

	incrementArm();
	REQUIRE(numCANPackets() == 1);
	p = popCANPacket();
	REQUIRE(GetDeviceSerialNumber(&p) == 1);
	REQUIRE(PacketIsOfID(&p, ID_MOTOR_UNIT_PID_POS_TGT_SET));
	REQUIRE(GetPIDTargetFromPacket(&p) == start_angle + 2 * expected_increment);

	// Artificially trigger the timeout
	Globals::status_data["arm_base"]["most_recent_command"] = 0;
	incrementArm();
	REQUIRE(numCANPackets() == 0);
	REQUIRE(Globals::status_data["arm_base"]["millideg_per_control_loop"] == 0);

	// Should clear incremental PID state if we switch back to PWM
	msg = "{\"type\": \"motor\", \"motor\": \"arm_base\", \"PWM target\":-0.5}";
	REQUIRE(ParseBaseStationPacket(msg) == true);
	REQUIRE(Globals::status_data["arm_base"]["millideg_per_control_loop"].is_null());
	REQUIRE(Globals::status_data["arm_base"]["target_angle"].is_null());
}

TEST_CASE("Does not allow incremental PID for non-PID motors", "[BaseStation]") {
	clearTestGlobals();
	setupEncoders();
	char const* msg =
		"{\"type\": \"motor\", \"motor\": \"hand\", \"incremental PID speed\":1.0}";
	REQUIRE(ParseBaseStationPacket(msg) == false);
	json m = json::parse(popBaseStationPacket());
	REQUIRE(m["msg"] == "Invalid operation mode 'incremental PID speed' for motor hand");
}

TEST_CASE("Can handle drive packets", "[BaseStation]") {
	char const* msg = "{\"type\":\"drive\",\"forward_backward\":0.3,\"left_right\":-0.4}";
	REQUIRE(ParseBaseStationPacket(msg) == true);
	REQUIRE(numCANPackets() == 4);
	for (uint8_t i = 8; i < 12; i++) {
		CANPacket p = popCANPacket();
		// We expect the packets to be sent in ascending order, starting with
		// the front-left wheel.
		REQUIRE(GetDeviceSerialNumber(&p) == i);
		REQUIRE(PacketIsOfID(&p, ID_MOTOR_UNIT_PWM_DIR_SET));
	}
}

TEST_CASE("Can handle malformed drive packets", "[BaseStation]") {
	char const* msg = "{\"type\":\"drive\",\"forward_backward\":2.9}";
	REQUIRE(ParseBaseStationPacket(msg) == false);
}

CANPacket popPIDPkt() {
	CANPacket p;
	while (!PacketIsOfID(&p, ID_MOTOR_UNIT_PID_POS_TGT_SET))
		p = popCANPacket();
	return p;
}

void assert_IK_equals(double base, double shoulder, double elbow) {
	CANPacket p;
	p = popPIDPkt();
	int base_val = DecodeBytesToIntMSBFirst(p.data, 1, 5);
	REQUIRE(intToRad(base_val, 1) == Approx(base).margin(0.01));

	p = popPIDPkt();
	int shoulder_val = DecodeBytesToIntMSBFirst(p.data, 1, 5);
	REQUIRE(intToRad(shoulder_val, 2) == Approx(shoulder).margin(0.01));

	p = popPIDPkt();
	int elbow_val = DecodeBytesToIntMSBFirst(p.data, 1, 5);
	REQUIRE(intToRad(elbow_val, 3) == Approx(elbow).margin(0.01));
}

void set_radian_arm_angles(double arm_base, double shoulder, double elbow) {
	Globals::status_data["arm_base"]["angular_position"] = radToInt(arm_base, 1);
	Globals::status_data["shoulder"]["angular_position"] = radToInt(shoulder, 2);
	Globals::status_data["elbow"]["angular_position"] = radToInt(elbow, 3);
}

void assert_FK_equals(double x, double y, double z) {
	std::array<double, 3> xyz = forward_kinematics();
	REQUIRE(x == Approx(xyz[0]).margin(0.01));
	REQUIRE(y == Approx(xyz[1]).margin(0.01));
	REQUIRE(z == Approx(xyz[2]).margin(0.01));
}

TEST_CASE("radToInt and intToRad", "[BaseStation]") {
	for (int i = -360 * 1000; i < 360 * 1000; i += 56565) {
		for (int serial = 1; serial < 4; serial++) {
			// printf("%d (serial %d) %f %d\n", i, serial, intToRad(i, serial),
			// radToInt(intToRad(i, serial), serial));
			REQUIRE(abs(radToInt(intToRad(i, serial), serial) - i) < 5);
		}
	}
}

TEST_CASE("Forward kinematics in stowed position", "[BaseStation]") {
	Globals::status_data["arm_base"]["angular_position"] = 0;
	Globals::status_data["shoulder"]["angular_position"] = 0;
	Globals::status_data["elbow"]["angular_position"] = 0;
	assert_FK_equals(0.119867, 0., 0.0152843);
}

TEST_CASE("Forward kinematics at full extension", "[BaseStation]") {
	double full_extension = Constants::SHOULDER_LENGTH + Constants::ELBOW_LENGTH;

	set_radian_arm_angles(0., 0., 0.);
	assert_FK_equals(full_extension, 0., 0.);

	set_radian_arm_angles(M_PI / 2, 0., 0.);
	assert_FK_equals(0., full_extension, 0.);

	set_radian_arm_angles(0., M_PI / 2, 0.);
	assert_FK_equals(0., 0., full_extension);
}

TEST_CASE("Forward kinematics near the ground", "[BaseStation]") {
	set_radian_arm_angles(0.9, 1.1, 2.3);
	assert_FK_equals(0.326845, 0.411873, -0.117700);
}

TEST_CASE("Can handle IK packets", "[BaseStation]") {
	clearTestGlobals();
	setupEncoders();
	char const* msg = "{\"type\":\"ik\",\"wrist_base_target\":[1.2, 0.0, 0.0]}";
	REQUIRE(ParseBaseStationPacket(msg) == true);
	assert_IK_equals(0., 0.428172, 0.792013);
}

TEST_CASE("Can reach targets below the ground plane", "[BaseStation]") {
	clearTestGlobals();
	setupEncoders();
	char const* msg = "{\"type\":\"ik\",\"wrist_base_target\":[0.6, 0.8, -0.2]}";
	REQUIRE(ParseBaseStationPacket(msg) == true);
	assert_IK_equals(0.927291, 0.534880, 1.342617);
}

// Not sure if we actually need the rover to do this during the competition,
// but it doesn't hurt to be prepared.
// This also helps make sure the robot doesn't go too crazy near the vertical pole?
TEST_CASE("Can reach some targets behind the vertical", "[BaseStation]") {
	clearTestGlobals();
	setupEncoders();
	char const* msg = "{\"type\":\"ik\",\"wrist_base_target\":[-0.1, 0.0, 1.26]}";
	REQUIRE(ParseBaseStationPacket(msg) == true);
	assert_IK_equals(0., 1.905204, 0.473425);
}

TEST_CASE("Can handle degenerate IK packets", "[BaseStation]") {
	setupEncoders();
	char const* msg = "{\"type\":\"ik\"}";
	REQUIRE(ParseBaseStationPacket(msg) == true);
	msg = "{\"type\":\"ik\",\"wrist_base_target\":\"asdf\"}";
	REQUIRE(ParseBaseStationPacket(msg) == false);
}

TEST_CASE("Reports error if IK target is infeasible", "[BaseStation]") {
	clearTestGlobals();
	setupEncoders();
	char const* msg = "{\"type\":\"ik\",\"wrist_base_target\":[100.0, 0.0, 0.0]}";
	REQUIRE(ParseBaseStationPacket(msg) == false);
	json m = json::parse(popBaseStationPacket());
	REQUIRE(m["msg"] == "Infeasible IK target");
}

TEST_CASE("Respects joint limits", "[BaseStation]") {
	clearTestGlobals();
	setupEncoders();
	char const* msg = "{\"type\":\"ik\",\"wrist_base_target\":[-1.0, 0.0, 0.0]}";
	REQUIRE(ParseBaseStationPacket(msg) == false);
	json m = json::parse(popBaseStationPacket());
	REQUIRE(m["msg"] == "IK solution outside joint limits for shoulder");
}

TEST_CASE("Forward kinematics is the inverse of inverse kinematics", "[BaseStation]") {
	clearTestGlobals();
	setupEncoders();
	char const* msg = "{\"type\":\"ik\",\"wrist_base_target\":[0.6, -0.3, -0.1]}";
	REQUIRE(ParseBaseStationPacket(msg) == true);
	assert_IK_equals(-0.463647, 1.005327, 2.053642);
	set_radian_arm_angles(-0.463647, 1.005327, 2.053642);
	assert_FK_equals(0.6, -0.3, -0.1);
}

TEST_CASE("Deactivates wheel motors if e-stopped", "e-stop") {
	clearTestGlobals();
	char const* estop_msg = "{\"type\":\"estop\",\"release\":false}";
	char const* drive_msg =
		"{\"type\":\"drive\",\"forward_backward\":0.3,\"left_right\":-0.4}";
	json m;
	CANPacket p;
	REQUIRE(ParseBaseStationPacket(drive_msg) == true);
	p = popCANPacket();
	REQUIRE(PacketIsOfID(&p, ID_MOTOR_UNIT_PWM_DIR_SET));
	REQUIRE(GetPWMFromPacket(&p) != 0);

	clearTestGlobals();
	REQUIRE(ParseBaseStationPacket(estop_msg) == true);
	m = json::parse(popBaseStationPacket());
	REQUIRE(m["status"] == "ok");
	// Should send broadcast e-stop
	p = popCANPacket();
	REQUIRE(PacketIsOfID(&p, ID_ESTOP));
	REQUIRE(GetDeviceGroupCode(&p) == DEVICE_GROUP_MOTOR_CONTROL);
	REQUIRE(GetDeviceSerialNumber(&p) == 0x0); // broadcast
	// Should also send individual motor e-stops
	for (uint8_t serial = 1; serial < 12; serial++) {
		p = popCANPacket();
		REQUIRE(PacketIsOfID(&p, ID_ESTOP));
		REQUIRE(GetDeviceGroupCode(&p) == DEVICE_GROUP_MOTOR_CONTROL);
		REQUIRE(GetDeviceSerialNumber(&p) == serial);
	}
	// Should also set wheel motor PWM to zero (mostly for use in simulation)
	p = popCANPacket();
	REQUIRE(PacketIsOfID(&p, ID_MOTOR_UNIT_PWM_DIR_SET));
	REQUIRE(GetPWMFromPacket(&p) == 0);

	clearTestGlobals();
	REQUIRE(ParseBaseStationPacket(drive_msg) == false);
	m = json::parse(popBaseStationPacket());
	REQUIRE(m["msg"] == "Emergency stop is activated");
	REQUIRE(numCANPackets() == 0);
}

TEST_CASE("Does not allow setCmdVel with nonzero values if e-stopped", "e-stop") {
	clearTestGlobals();
	char const* estop_msg = "{\"type\":\"estop\",\"release\":false}";
	REQUIRE(ParseBaseStationPacket(estop_msg) == true);
	clearTestGlobals();
	REQUIRE(setCmdVel(0.0, 0.1) == false);
	REQUIRE(numCANPackets() == 0);
	REQUIRE(setCmdVel(-0.1, 0.0) == false);
	REQUIRE(numCANPackets() == 0);
	REQUIRE(setCmdVel(0.0, 0.0) == true);
	REQUIRE(numCANPackets() == 4);
}
