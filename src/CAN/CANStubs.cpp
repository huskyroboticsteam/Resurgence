
#include "CAN.h"

#include <iostream>

namespace can {
namespace {
uint32_t callbackID = 0;
};

void initCAN() {}

void sendCANPacket(const CANPacket& packet) {}

robot::types::DataPoint<telemetry_t> getDeviceTelemetry(deviceid_t id, telemtype_t telemType) {
	return {};
}

callbackid_t addDeviceTelemetryCallback(
	deviceid_t id, telemtype_t telemType,
	const std::function<void(deviceid_t, telemtype_t, robot::types::DataPoint<telemetry_t>)>&
		callback) {
	return {id, telemType, callbackID++};
}

void removeDeviceTelemetryCallback(callbackid_t id) {}
} // namespace can
