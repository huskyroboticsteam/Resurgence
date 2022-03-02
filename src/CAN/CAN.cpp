#include "CAN.h"

#include "../log.h"
#include "CANUtils.h"

#include <cstring>
#include <map>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unistd.h>
#include <utility>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

extern "C" {
#include "../HindsightCAN/CANCommon.h"
}

namespace can {
namespace {
int can_fd;
sockaddr_can can_addr;
std::mutex socketMutex; // protects both can_fd and can_addr

using telemetrycode_t = uint8_t;

// the telemetry map will store telemetry code instead of telem enum
// this means unrecognized telemetry types won't cause UB
using protectedmap_t = std::pair<std::shared_ptr<std::shared_mutex>,
								 std::shared_ptr<std::map<telemetrycode_t, int32_t>>>;

std::map<deviceid_t, protectedmap_t> telemMap;
std::shared_mutex telemMapMutex;

void error(const std::string& err) {
	std::perror(err.c_str());
	std::exit(1);
}

void recievePacket(CANPacket& packet) {
	socklen_t len = sizeof(can_addr);

	bool success;
	can_frame frame;
	{
		// lock the socket only while we're using it
		std::lock_guard lock(socketMutex);
		// no need to check for EAGAIN or EWOULDBLOCK since we're blocking
		success = recvfrom(can_fd, &frame, sizeof(struct can_frame), 0,
						   (struct sockaddr*)&can_addr, &len) >= 0;
	}
	if (success) {
		log(LOG_TRACE, "Got CAN packet\n");
		packet.id = frame.can_id;
		packet.dlc = frame.can_dlc;
		for (int i = 0; i < frame.can_dlc; i++) {
			packet.data[i] = frame.data[i];
		}
	} else {
		std::perror("Failed to receive CAN packet!");
	}
}

template <typename K, typename V>
bool mapHasKey(std::shared_mutex& mutex, const std::map<K, V>& map, const K& key) {
	std::shared_lock lock(mutex);
	return map.find(key) != map.end();
}

void recieveThreadFn() {
	CANPacket packet;

	while (true) {
		recievePacket(packet);

		// extract necessary information
		deviceid_t id = getDeviceGroupAndSerial(packet);
		telemetrycode_t telemCode = DecodeTelemetryType(&packet);
		int32_t telemData = DecodeTelemetryDataSigned(&packet);

		// check if telemetry data is alread in map
		if (mapHasKey(telemMapMutex, telemMap, id)) {
			// acquire read lock of entire map
			std::shared_lock mapLock(telemMapMutex);
			auto& pair = telemMap.at(id);
			std::shared_mutex& deviceMutex = *pair.first;
			auto& deviceMap = *pair.second;
			// acquire write lock of the map for this device
			std::unique_lock deviceLock(deviceMutex);
			// insert telemetry data
			deviceMap.insert(std::make_pair(telemCode, telemData));
		} else {
			// this device has no existing data, so insert a new device map
			auto mutexPtr = std::make_shared<std::shared_mutex>();
			auto deviceMapPtr = std::make_shared<std::map<telemtype_t, int32_t>>();
			deviceMapPtr->emplace(telemCode, telemData);
			// acquire write lock of the entire map to insert a new device map
			std::unique_lock mapLock(telemMapMutex);
			telemMap.emplace(id, std::make_pair(mutexPtr, deviceMapPtr));
		}
	}
}
} // namespace

void initCAN() {
	std::lock_guard lock(socketMutex);
	if ((can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		error("Failed to initialize CAN bus!");
	}

	ifreq can_ifr;
	std::strcpy(can_ifr.ifr_name, "can0");
	if (ioctl(can_fd, SIOCGIFINDEX, &can_ifr) < 0) {
		std::perror("Failed to get hardware CAN interface index\n"
					"You can enable CAN with\n\n"
					"  ~/Resurgence/enable_CAN_and_GPS.sh\n\n");
		std::strcpy(can_ifr.ifr_name, "vcan0");
		if (ioctl(can_fd, SIOCGIFINDEX, &can_ifr) < 0) {
			error("Failed to get virtual CAN interface index\n"
				  "You can enable CAN with\n\n"
				  "  ~/Resurgence/enable_CAN_and_GPS.sh\n\n");
		}
		log(LOG_INFO, "Found virtual CAN interface index.\n");
	}

	can_addr.can_family = AF_CAN;
	log(LOG_DEBUG, "Index: %d\n", can_ifr.ifr_ifindex);
	can_addr.can_ifindex = can_ifr.ifr_ifindex;

	if (bind(can_fd, (struct sockaddr*)&can_addr, sizeof(can_addr)) < 0) {
		error("Failed to bind CAN socket");
	}

	// start thread for recieving CAN packets
	std::thread recieveThread(recieveThreadFn);
	recieveThread.detach();
}

void sendCANPacket(const CANPacket& packet) {
	can_frame frame;
	frame.can_id = packet.id;
	frame.can_dlc = packet.dlc;
	for (int i = 0; i < packet.dlc; i++) {
		frame.data[i] = packet.data[i];
	}

	bool success;
	{
		std::lock_guard lock(socketMutex);
		// not marked as nonblocking, so we shouldn't see EAGAIN or EWOULDBLOCK
		success = sendto(can_fd, &frame, sizeof(struct can_frame), 0,
						 (struct sockaddr*)&can_addr, sizeof(can_addr)) >= 0;
	}
	if (success) {
		log(LOG_TRACE, "CAN packet sent.\n");
	} else {
		std::perror("Failed to send CAN packet");
	}
}

std::optional<telemetry_t> getDeviceTelemetry(deviceid_t id, telemtype_t telemType) {
	std::shared_lock mapLock(telemMapMutex);
	auto entry = telemMap.find(id);
	if (entry != telemMap.end()) {
		auto& devMutex = *entry->second.first;
		auto& devMap = *entry->second.second;
		std::shared_lock deviceLock(devMutex);
		auto telemEntry = devMap.find(static_cast<telemetrycode_t>(telemType));
		if (telemEntry != devMap.end()) {
			return telemEntry->second;
		} else {
			return {};
		}
	} else {
		return {};
	}
}

} // namespace can
