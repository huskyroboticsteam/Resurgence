
#include "CAN.h"

#include <iostream>

namespace can {
namespace {
uint32_t callbackID = 0;
};

void initCAN() {}

void sendCANPacket(const CANPacket_t& packet) {}
// void sendCANPacket(const CANPacket& packet) {}

void printCANPacket(const CANPacket_t& packet) {}

void pullDeviceTelemetry(CANDeviceUUID_t uuid, telemtype_t telemType) {}

void scheduleTelemetryPull(CANDeviceUUID_t uuid, telemtype_t telemType,
						   std::chrono::milliseconds period) {}

void unscheduleTelemetryPull(CANDeviceUUID_t uuid, telemtype_t telemType) {}

void unscheduleAllTelemetryPulls() {}

robot::types::DataPoint<telemetry_t> getDeviceTelemetry(CANDeviceUUID_t id, telemtype_t telemType) {
	return {};
}
/*
robot::types::DataPoint<telemetry_t> getDeviceTelemetry(deviceid_t id, telemtype_t telemType) {
	return {};
}
*/

callbackid_t addDeviceTelemetryCallback(
	CANDeviceUUID_t uuid, telemtype_t telemType,
	const std::function<void(CANDeviceUUID_t, telemtype_t, robot::types::DataPoint<telemetry_t>)>&
		callback) {
	return {uuid, telemType, callbackID++};
}
/*
callbackid_t addDeviceTelemetryCallback(
	deviceid_t id, telemtype_t telemType,
	const std::function<void(deviceid_t, telemtype_t, robot::types::DataPoint<telemetry_t>)>&
		callback) {
	return {id, telemType, callbackID++};
}
*/

void removeDeviceTelemetryCallback(callbackid_t id) {}
} // namespace can
