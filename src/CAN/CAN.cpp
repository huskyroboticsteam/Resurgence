#include "CAN.h"

#include "../log.h"
#include "CANUtils.h"

#include <chrono>
#include <cstring>
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

namespace can {
namespace {
constexpr std::chrono::milliseconds READ_ERR_SLEEP(100);
// receive all messages on the given group
constexpr int CAN_MASK = (0 << 10) | (0b1111 << 6) | 0;

int can_fd;
std::mutex socketMutex; // protects can_fd

using telemetrycode_t = uint8_t;

// the telemetry map will store telemetry code instead of telem enum
// this means unrecognized telemetry types won't cause UB
using protectedmap_t =
	std::pair<std::shared_ptr<std::shared_mutex>,
			  std::shared_ptr<std::map<telemetrycode_t, DataPoint<telemetry_t>>>>;

std::map<deviceid_t, protectedmap_t> telemMap;
std::shared_mutex telemMapMutex;

// not thread-safe wrt file descriptor
bool receivePacket(int fd, CANPacket& packet) {
	int ret;
	can_frame frame;
	ret = read(fd, &frame, sizeof(can_frame));
	if (ret >= 0) {
		log(LOG_TRACE, "Got CAN packet\n");
		packet.id = frame.can_id;
		packet.dlc = frame.can_dlc;
		std::memcpy(packet.data, frame.data, frame.can_dlc);
		return true;
	} else {
		std::perror("Failed to receive CAN packet!");
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

// returns a file descriptor, or -1 on failure
int createCANSocket(std::optional<can::deviceid_t> id) {
	int fd;
	if ((fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		std::perror("Failed to initialize CAN bus!");
		return -1;
	}

	struct ifreq ifr;
	std::strcpy(ifr.ifr_name, "can0");
	if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
		std::perror("Failed to get hardware CAN interface index");
		std::strcpy(ifr.ifr_name, "vcan0");
		if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
			std::perror("Failed to get virtual CAN interface index");
			return -1;
		}
		log(LOG_INFO, "Found virtual CAN interface index.\n");
	}

	struct sockaddr_can addr;
	std::memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
		std::perror("Bind");
		return -1;
	}

	// enable reception at the given id, if provided
	if (id) {
		int canID = ConstructCANID(0, static_cast<uint8_t>(id->first), id->second);
		can_filter filters[1];
		filters[0].can_id = canID;
		filters[0].can_mask = CAN_MASK;

		setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filters, sizeof(filters));
	} else {
		// disable reception on this socket.
		setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0);
	}

	return fd;
}

void receiveThreadFn() {
	CANPacket packet;
	// create dedicated CAN socket for reading
	int recvFD = createCANSocket({{devicegroup_t::master, DEVICE_SERIAL_JETSON}});
	if (recvFD < 0) {
		log(LOG_ERROR, "Unable to open CAN connection!\n");
		return;
	}

	while (true) {
		// no sychronization necessary, since this thread owns the FD
		bool received = receivePacket(recvFD, packet);
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
			// we had a bus error, so sleep for a bit
			std::this_thread::sleep_for(READ_ERR_SLEEP);
		}
	}
}
} // namespace

void initCAN() {
	std::lock_guard lock(socketMutex);
	can_fd = createCANSocket({});
	if (can_fd < 0) {
		log(LOG_ERROR, "Unable to open CAN connection!\n");
	}

	// start thread for recieving CAN packets
	std::thread receiveThread(receiveThreadFn);
	receiveThread.detach();
}

void sendCANPacket(const CANPacket& packet) {
	can_frame frame;
	frame.can_id = packet.id;
	frame.can_dlc = packet.dlc;
	std::memcpy(frame.data, packet.data, packet.dlc);
	bool success;
	{
		std::lock_guard lock(socketMutex);
		success = write(can_fd, &frame, sizeof(struct can_frame)) == sizeof(struct can_frame);
		tcdrain(can_fd);
	}

	if (!success) {
		std::perror("Failed to send CAN packet");
	} else {
		log(LOG_TRACE, "CAN packet sent.\n");
	}
}

robot::types::DataPoint<telemetry_t> getDeviceTelemetry(deviceid_t id, telemtype_t telemType) {
	std::shared_lock mapLock(telemMapMutex); // acquire read lock
	// find entry for device in map
	auto entry = telemMap.find(id);
	if (entry != telemMap.end()) {
		auto& devMutex = *entry->second.first;
		auto& devMap = *entry->second.second;
		// acquire read lock of device map
		std::shared_lock deviceLock(devMutex);
		// find entry for telemetry
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
