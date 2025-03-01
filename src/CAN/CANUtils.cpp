#include "CANUtils.h"

#include "CAN.h"

#include <ios>
#include <sstream>
#include <thread>

extern "C" {
#include <HindsightCAN/CANPacket.h>
}

namespace can {
devicegroup_t getDeviceGroup(const CANPacket& packet) {
	uint8_t groupCode = GetDeviceGroupCode(const_cast<CANPacket*>(&packet));
	return static_cast<devicegroup_t>(groupCode);
}

deviceserial_t getDeviceSerial(const CANPacket& packet) {
	return GetDeviceSerialNumber(const_cast<CANPacket*>(&packet));
}

deviceid_t getDeviceGroupAndSerial(const CANPacket& packet) {
	return std::make_pair(getDeviceGroup(packet), getDeviceSerial(packet));
}

deviceserial_t getSenderDeviceSerial(const CANPacket& packet) {
	return GetSenderDeviceSerialNumber(const_cast<CANPacket*>(&packet));
}

devicegroup_t getSenderDeviceGroup(const CANPacket& packet) {
	uint8_t groupCode = GetSenderDeviceGroupCode(const_cast<CANPacket*>(&packet));
	return static_cast<devicegroup_t>(groupCode);
}

deviceid_t getSenderDeviceGroupAndSerial(const CANPacket& packet) {
	return std::make_pair(getSenderDeviceGroup(packet), getSenderDeviceSerial(packet));
}

std::string packetToString(const CANPacket& packet) {
	std::stringstream ss;
	ss << std::hex << packet.id << "#";
	for (int i = 0; i < packet.dlc; i++) {
		ss << std::hex << static_cast<int>(packet.data[i]);
		if (i < packet.dlc - 1) {
			ss << ".";
		}
	}
	return ss.str();
}

bool hasData(deviceid_t id) {
	auto start = std::chrono::steady_clock::now();
	constexpr int timeout_ms = 100;

	while (std::chrono::duration_cast<std::chrono::milliseconds>(
			   std::chrono::steady_clock::now() - start)
			   .count() < timeout_ms) {
		LOG_F(INFO, std::to_string(scheduleTelemetryPull(id, telemtype_t::adc_raw,
											   std::chrono::milliseconds(100))).c_str());
		if (getDeviceTelemetry(id, telemtype_t::adc_raw).isValid()) {
			return true;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	return false;
}

void verifyAllMotorsConnected(
	frozen::unordered_map<robot::types::motorid_t, deviceserial_t, 18UL> motor_id_map) {
	std::vector<deviceserial_t> disconnected_motors;
	for (auto motorMap : motor_id_map) {
		auto motor_pair = std::make_pair(devicegroup_t::motor, motorMap.second);
		pullDeviceTelemetry(motor_pair, telemtype_t::voltage);
		if (!hasData(motor_pair)) {
			disconnected_motors.push_back(motorMap.second);
		}
	}

	if (disconnected_motors.empty()) {
		LOG_F(INFO, "All motors connected!");
	} else {
		LOG_F(ERROR, "Disconnected motors:");
		for (auto motor_id : disconnected_motors) {
			LOG_F(ERROR, std::to_string(motor_id).c_str());
		}
		throw std::runtime_error("Not all motors connected!");
	}
}

} // namespace can
