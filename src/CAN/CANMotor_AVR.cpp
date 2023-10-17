#include "CAN.h"
#include "CANMotor.h"
#include "CANUtils.h"

#include <chrono>
#include <thread>

extern "C" {
#include <HindsightCAN/CANCommon.h>
#include <HindsightCAN/CANMotorUnit.h>
#include <HindsightCAN/CANPacket.h>
}

using namespace std::chrono_literals;

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
		AssembleTelemetryTimingPacket(&p, motorGroupCode, serial,
									  PACKET_TELEMETRY_ANG_POSITION,
									  telemetryPeriod.value().count());
		sendCANPacket(p);
		std::this_thread::sleep_for(1000us);
	}
}
} // namespace can::motor
