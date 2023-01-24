#include "CAN.h"
#include "CANMotor.h"
#include "CANUtils.h"

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

extern "C" {
#include <HindsightCAN/CANCommon.h>
#include <HindsightCAN/CANMotorUnit.h>
#include <HindsightCAN/CANPacket.h>
}

using namespace std::chrono_literals;
using std::chrono::milliseconds;

namespace can::motor {

void initEncoder(deviceserial_t serial, bool invertEncoder, bool zeroEncoder,
				 int32_t pulsesPerJointRev,
				 std::optional<std::chrono::milliseconds> telemetryPeriod) {
	auto motorGroupCode = static_cast<uint8_t>(devicegroup_t::motor);
	CANPacket p;
	AssembleEncoderInitializePacket(&p, motorGroupCode, serial, sensor_t::encoder,
									invertEncoder, zeroEncoder);
	sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
	AssembleEncoderPPJRSetPacket(&p, motorGroupCode, serial, pulsesPerJointRev);
	sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
	if (telemetryPeriod) {
		scheduleTelemetryPull(std::make_pair(devicegroup_t::motor, serial), telemtype_t::angle,
							  telemetryPeriod.value());
	}
}

void initPotentiometer(deviceserial_t serial, int32_t posLo, int32_t posHi, uint16_t adcLo,
					   uint16_t adcHi,
					   std::optional<std::chrono::milliseconds> telemetryPeriod) {
	CANPacket packet;
	auto group = static_cast<uint8_t>(devicegroup_t::motor);
	AssemblePotHiSetPacket(&packet, group, serial, adcHi, posHi);
	sendCANPacket(packet);
	AssemblePotLoSetPacket(&packet, group, serial, adcLo, posLo);
	sendCANPacket(packet);
	if (telemetryPeriod) {
		scheduleTelemetryPull(std::make_pair(devicegroup_t::motor, serial), telemtype_t::angle,
							  telemetryPeriod.value());
	}
}
} // namespace can::motor
