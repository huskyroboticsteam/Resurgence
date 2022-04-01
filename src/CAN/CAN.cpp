#include "CAN.h"

#include "../log.h"
#include "CANUtils.h"

#include <cstring>
#include <chrono>
#include <map>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <termios.h>
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

using robot::types::DataPoint;

constexpr std::chrono::milliseconds READ_PERIOD(10);

namespace can {
// namespace {
int can_fd;
sockaddr_can can_addr;
std::mutex socketMutex; // protects both can_fd and can_addr

using telemetrycode_t = uint8_t;

// the telemetry map will store telemetry code instead of telem enum
// this means unrecognized telemetry types won't cause UB
using protectedmap_t =
	std::pair<std::shared_ptr<std::shared_mutex>,
			  std::shared_ptr<std::map<telemetrycode_t, DataPoint<telemetry_t>>>>;

std::map<deviceid_t, protectedmap_t> telemMap;
std::shared_mutex telemMapMutex;

void error(const std::string& err) {
	std::perror(err.c_str());
	std::exit(1);
}

bool receivePacket(CANPacket& packet) {
	socklen_t len = sizeof(can_addr);

	int ret;
	can_frame frame;
	{
		// lock the socket only while we're using it
		std::lock_guard lock(socketMutex);
		// we won't loop if we get EAGAIN or EWOULDBLOCK
		ret = recvfrom(can_fd, &frame, sizeof(can_frame), MSG_DONTWAIT,
						   reinterpret_cast<sockaddr*>(&can_addr), &len);
	}
	if (ret >= 0) {
		log(LOG_TRACE, "Got CAN packet\n");
		packet.id = frame.can_id;
		packet.dlc = frame.can_dlc;
		for (int i = 0; i < frame.can_dlc; i++) {
			packet.data[i] = frame.data[i];
		}
		return true;
	} else if (ret != EAGAIN && ret != EWOULDBLOCK) {
		std::perror("Failed to receive CAN packet!");
		return false;
	} else {
		return false;
	}
}

template <typename K, typename V>
bool mapHasKey(std::shared_mutex& mutex, const std::map<K, V>& map, const K& key) {
	std::shared_lock lock(mutex);
	return map.find(key) != map.end();
}

void handleTelemetryPacket(CANPacket& packet) {
	// extract necessary information
	deviceid_t id = getSenderDeviceGroupAndSerial(packet);
	telemetrycode_t telemCode = DecodeTelemetryType(&packet);
	telemetry_t telemData = DecodeTelemetryDataSigned(&packet);
	DataPoint<telemetry_t> data(telemData);

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
		deviceMap.insert(std::make_pair(telemCode, data));
	} else {
		// this device has no existing data, so insert a new device map
		auto mutexPtr = std::make_shared<std::shared_mutex>();
		auto deviceMapPtr =
			std::make_shared<std::map<telemetrycode_t, DataPoint<telemetry_t>>>();
		deviceMapPtr->emplace(telemCode, data);
		// acquire write lock of the entire map to insert a new device map
		std::unique_lock mapLock(telemMapMutex);
		telemMap.emplace(id, std::make_pair(mutexPtr, deviceMapPtr));
	}
}

void receiveThreadFn() {
	CANPacket packet;

	while (true) {
		bool received = receivePacket(packet);
		if (received) {
			uint8_t packetType = packet.data[0];
			switch (packetType) {
				case packettype_t::telemetry:
					handleTelemetryPacket(packet);
					break;

				default:
					log(LOG_WARN, "Unrecognized CAN packet type: %x\n", packetType);
					break;
			}
		} else {
			std::this_thread::sleep_for(READ_PERIOD);
		}
	}
}
// } // namespace

void initCAN() {
	// int s;

	if ((can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Socket");
		return;
	}

	struct ifreq ifr;
	strcpy(ifr.ifr_name, "can0");
	ioctl(can_fd, SIOCGIFINDEX, &ifr);

	struct sockaddr_can addr;
	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(can_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
		perror("Bind");
		return;
	}

	// std::lock_guard lock(socketMutex);
	// if ((can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
	// 	error("Failed to initialize CAN bus!");
	// }

	// ifreq can_ifr;
	// std::strcpy(can_ifr.ifr_name, "can0");
	// if (ioctl(can_fd, SIOCGIFINDEX, &can_ifr) < 0) {
	// 	std::perror("Failed to get hardware CAN interface index\n"
	// 				"You can enable CAN with\n\n"
	// 				"  ~/Resurgence/enable_CAN_and_GPS.sh\n\n");
	// 	std::strcpy(can_ifr.ifr_name, "vcan0");
	// 	if (ioctl(can_fd, SIOCGIFINDEX, &can_ifr) < 0) {
	// 		error("Failed to get virtual CAN interface index\n"
	// 			  "You can enable CAN with\n\n"
	// 			  "  ~/Resurgence/enable_CAN_and_GPS.sh\n\n");
	// 	}
	// 	log(LOG_INFO, "Found virtual CAN interface index.\n");
	// }

	// std::memset(&can_addr, 0, sizeof(can_addr));
	// can_addr.can_family = AF_CAN;
	// log(LOG_DEBUG, "Index: %d\n", can_ifr.ifr_ifindex);
	// can_addr.can_ifindex = can_ifr.ifr_ifindex;

	// if (bind(can_fd, (struct sockaddr*)&can_addr, sizeof(can_addr)) < 0) {
	// 	error("Failed to bind CAN socket");
	// }

	// start thread for recieving CAN packets
	// std::thread receiveThread(receiveThreadFn);
	// receiveThread.detach();
}

void sendCANPacket(const CANPacket& packet) {
	can_frame frame;
	frame.can_id = packet.id;
	frame.can_dlc = packet.dlc;
	std::memcpy(frame.data, packet.data, packet.dlc);

	if (write(can_fd, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
		std::perror("Failed to send CAN packet");
	} else {
		log(LOG_TRACE, "CAN packet sent.\n");
	}

	tcdrain(can_fd);

	// bool success;
	// {
	// 	std::lock_guard lock(socketMutex);
	// 	// not marked as nonblocking, so we shouldn't see EAGAIN or EWOULDBLOCK
	// 	success = sendto(can_fd, &frame, sizeof(struct can_frame), 0,
	// 					 (struct sockaddr*)&can_addr, sizeof(can_addr)) >= 0;
	// }
	// if (success) {
	// 	log(LOG_TRACE, "CAN packet sent.\n");
	// } else {
	// 	std::perror("Failed to send CAN packet");
	// }
}

robot::types::DataPoint<telemetry_t> getDeviceTelemetry(deviceid_t id, telemtype_t telemType) {
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
