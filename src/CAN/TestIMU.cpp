#include "CAN.h"
extern "C" {
#include <HindsightCAN/CANCommon.h>
#include <HindsightCAN/CANSerialNumbers.h>
}

#include <chrono>
#include <loguru.hpp>
#include <thread>

using namespace std::chrono_literals;
using namespace robot::types;

constexpr can::devicegroup_t IMU_GROUP = can::devicegroup_t::telemetry;
constexpr can::deviceserial_t IMU_SERIAL = DEVICE_SERIAL_TELEM_IMU;
constexpr auto LOOP_PERIOD = 100ms;

constexpr can::telemtype_t IMU_DATA_TYPES[] = {
	can::telemtype_t::gyro_x, can::telemtype_t::gyro_y, can::telemtype_t::gyro_z
	// TODO: add the quat telemetry types (once we figure out the telemetry codes) and add to
	// this array
};

void telemCallback(can::deviceid_t id, can::telemtype_t telemType,
				   DataPoint<can::telemetry_t> data) {
	if (data) {
		LOG_F(INFO, "Telemetry: TelemType=%x, Data=%d\n", static_cast<int>(telemType),
			  data.getData());
	} else {
		LOG_F(INFO, "Telemetry: TelemType=%x, Data=null\n", static_cast<int>(telemType));
	}
}

int main() {
	can::initCAN();

	for (auto telemType : IMU_DATA_TYPES) {
		can::addDeviceTelemetryCallback(std::make_pair(IMU_GROUP, IMU_SERIAL), telemType,
										telemCallback);
	}

	while (true) {
		CANPacket packet;
		for (auto telemType : IMU_DATA_TYPES) {
			AssembleTelemetryPullPacket(&packet, static_cast<uint8_t>(IMU_GROUP), IMU_SERIAL,
										static_cast<uint8_t>(telemType));
			can::sendCANPacket(packet);
		}
		std::this_thread::sleep_for(LOOP_PERIOD);
	}
}
