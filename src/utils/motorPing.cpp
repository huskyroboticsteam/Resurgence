#include "../CAN/CAN.h"
#include "../CAN/CANUtils.h"
#include "../world_interface/real_world_constants.h"

#include <chrono>
#include <thread>

namespace motorPing {
bool hasData(deviceid_t id) {
	auto start = std::chrono::steady_clock::now();
	const int timeout_ms = 100;

	while (std::chrono::duration_cast<std::chrono::milliseconds>(
			   std::chrono::steady_clock::now() - start)
			   .count() < timeout_ms) {
		if (can::checkDeviceTelemetry(id)) {
			return true;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	return false;
}

void pingMotors() {
	for (auto motorMap : robot::motorSerialIDMap) {
		can::pullDeviceTelemetry(std::make_pair(devicegroup_t::motor, motorMap.second),
								 telemtype_t::voltage);
		if (!hasData(std::make_pair(devicegroup_t::motor, motorMap.second))) {
			LOG_F(ERROR, "Motor not connected!\nID: " + motorMap.first +
							 "\nSerial: " + motorMap.second);
			throw std::runtime_error("Motor not connected!\nID: " + motorMap.first +
									 "\nSerial: " + motorMap.second);
		}
	}
}
} // namespace motorPing
