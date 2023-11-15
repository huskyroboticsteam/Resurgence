#include "CAN.h"

#include "CANMotor.h"

#include <loguru.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

enum class testmode_t
{
	automatic = 0,
	autoSend = 1,
	manualSend = 2,
	fullManual = 3
};

constexpr uint8_t serial = 0xA;
constexpr int16_t pwm = 10000;

void automatic() {
	can::initCAN();
	can::motor::setMotorMode(serial, can::motor::motormode_t::pwm);
	can::motor::setMotorPower(serial, pwm);
}

void autoSend() {
	can::initCAN();
	CANPacket p;
	AssembleModeSetPacket(&p, 0x4, serial, 0x0);
	can::sendCANPacket(p);
	AssemblePWMDirSetPacket(&p, 0x4, serial, pwm);
	can::sendCANPacket(p);
}

void manualSend() {
	int s;

	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Socket");
		return;
	}

	struct ifreq ifr;
	strcpy(ifr.ifr_name, "can0");
	ioctl(s, SIOCGIFINDEX, &ifr);

	struct sockaddr_can addr;
	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
		perror("Bind");
		return;
	}

	{
		CANPacket p;
		AssembleModeSetPacket(&p, 0x4, serial, 0x0);
		can_frame frame;
		frame.can_id = p.id;
		frame.can_dlc = p.dlc;
		memcpy(frame.data, p.data, p.dlc);

		if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
			perror("Mode set error");
			return;
		}
	}
	{
		CANPacket p;
		AssemblePWMDirSetPacket(&p, 0x4, serial, pwm);
		can_frame frame;
		frame.can_id = p.id;
		frame.can_dlc = p.dlc;
		memcpy(frame.data, p.data, p.dlc);

		if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
			perror("PWM set error");
			return;
		}
	}

	if (close(s) < 0) {
		perror("Close");
		return;
	}
}

void fullManual() {
	int s;

	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Socket");
		return;
	}

	struct ifreq ifr;
	strcpy(ifr.ifr_name, "can0");
	ioctl(s, SIOCGIFINDEX, &ifr);

	struct sockaddr_can addr;
	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
		perror("Bind");
		return;
	}

	{
		can_frame frame;
		frame.can_id = 0x50a;
		frame.can_dlc = 2;
		uint8_t modesetData[] = {0x0, 0x0};
		memcpy(frame.data, modesetData, 2);

		if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
			perror("Mode set error");
			return;
		}
	}
	{
		can_frame frame;
		frame.can_id = 0x50a;
		frame.can_dlc = 3;
		uint8_t pwmsetData[] = {0x03, 0x27, 0x10};
		memcpy(frame.data, pwmsetData, 3);

		if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
			perror("PWM set error");
			return;
		}
	}

	if (close(s) < 0) {
		perror("Close");
		return;
	}
}

int main(int argc, char* argv[]) {
	testmode_t mode = testmode_t::automatic;

	if (argc >= 2) {
		int i = std::stoi(argv[1]);
		mode = static_cast<testmode_t>(i);
	}

	LOG_F(INFO, "Mode=%d", static_cast<int>(mode));

	switch (mode) {
		case testmode_t::automatic:
			automatic();
			break;
		case testmode_t::autoSend:
			autoSend();
			break;
		case testmode_t::manualSend:
			manualSend();
			break;
		case testmode_t::fullManual:
			fullManual();
			break;
	}
}