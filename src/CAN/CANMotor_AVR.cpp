#include "CAN.h"
#include "CANMotor.h"
#include "CANUtils.h"

#include <chrono>
#include <thread>

extern "C" {
#include "../HindsightCAN/CANCommon.h"
#include "../HindsightCAN/CANMotorUnit.h"
#include "../HindsightCAN/CANPacket.h"
}

using namespace std::chrono_literals;

namespace can::motor {
void initMotor(deviceserial_t serial, bool invertEncoder, bool zeroEncoder,
			   int32_t pulsesPerJointRev, std::chrono::milliseconds telemetryPeriod) {
	initMotor(serial);
	auto motorGroupCode = static_cast<uint8_t>(devicegroup_t::motor);
	CANPacket p;
	AssembleEncoderInitializePacket(&p, motorGroupCode, serial, 0, invertEncoder, zeroEncoder);
	sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
	AssembleEncoderPPJRSetPacket(&p, motorGroupCode, serial, pulsesPerJointRev);
	sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
	AssembleTelemetryTimingPacket(&p, motorGroupCode, serial, PACKET_TELEMETRY_ANG_POSITION,
								  telemetryPeriod.count());
	sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
}

void pullMotorPosition(deviceserial_t serial) {}
} // namespace can::motor
