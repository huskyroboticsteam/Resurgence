#include "CAN.h"

#include "../log.h"
#include "CANUtils.h"

#include <chrono>
#include <cstring>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>
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

// template specialization for hashing pairs
template <typename T1, typename T2> struct std::hash<std::pair<T1, T2>> {
	std::size_t operator()(const std::pair<T1, T2>& pair) const {
		auto h1 = std::hash<T1>()(pair.first);
		auto h2 = std::hash<T2>()(pair.second);
		// stolen from:
		// https://stackoverflow.com/questions/2590677/how-do-i-combine-hash-values-in-c0x
		return (h1 << 6) + (h1 >> 2) + h2 + 0x9e3779b9;
	}
};


namespace can {
namespace {
// time to sleep after getting a CAN read error
constexpr std::chrono::milliseconds READ_ERR_SLEEP(100);
// receive all messages on the given group
constexpr uint32_t CAN_MASK = (0 << 10) | (0b1111 << 6) | 0;

int can_fd;				// file descriptor of outbound can connection
std::mutex socketMutex; // protects can_fd

using telemetrycode_t = uint8_t;

const std::unordered_map<telemetrycode_t, telemtype_t> telemCodeToTypeMap = {
	{PACKET_TELEMETRY_VOLTAGE, telemtype_t::voltage},
	{PACKET_TELEMETRY_CURRENT, telemtype_t::current},
	{PACKET_TELEMETRY_PWR_RAIL_STATE, telemtype_t::pwr_rail},
	{PACKET_TELEMETRY_TEMPERATURE, telemtype_t::temp},
	{PACKET_TELEMETRY_ANG_POSITION, telemtype_t::angle},
	{PACKET_TELEMETRY_GPS_LAT, telemtype_t::gps_lat},
	{PACKET_TELEMETRY_GPS_LON, telemtype_t::gps_lon},
	{PACKET_TELEMETRY_MAG_DIR, telemtype_t::mag_dir},
	{PACKET_TELEMETRY_ACCEL_X, telemtype_t::accel_x},
	{PACKET_TELEMETRY_ACCEL_Y, telemtype_t::accel_y},
	{PACKET_TELEMETRY_ACCEL_Z, telemtype_t::accel_z},
	{PACKET_TELEMETRY_GYRO_X, telemtype_t::gyro_x},
	{PACKET_TELEMETRY_GYRO_Y, telemtype_t::gyro_y},
	{PACKET_TELEMETRY_GYRO_Z, telemtype_t::gyro_z},
	{PACKET_TELEMETRY_LIM_SW_STATE, telemtype_t::limit_switch}};

// the telemetry map will store telemetry code instead of telem enum
// this means unrecognized telemetry types won't cause UB
using devicemap_t =
	std::pair<std::shared_ptr<std::shared_mutex>,
			  std::shared_ptr<std::unordered_map<telemetrycode_t, DataPoint<telemetry_t>>>>;

// holds telemetry data for each device
std::unordered_map<deviceid_t, devicemap_t> telemMap;
std::shared_mutex telemMapMutex;

std::unordered_map<std::pair<deviceid_t, telemetrycode_t>,
		 std::unordered_map<uint32_t, std::function<void(deviceid_t, telemtype_t,
											   robot::types::DataPoint<telemetry_t>)>>>
	telemetryCallbackMap;
uint32_t nextCallbackID = 0;
std::mutex telemetryCallbackMapMutex; // protects callbackMap and callbackID

// not thread-safe wrt file descriptor
bool receivePacket(int fd, CANPacket& packet) {
	int ret;
	can_frame frame;
	ret = read(fd, &frame, sizeof(can_frame));
	if (ret >= 0) {
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
bool mapHasKey(std::shared_mutex& mutex, const std::unordered_map<K, V>& map, const K& key) {
	std::shared_lock lock(mutex);
	return map.find(key) != map.end();
}

void invokeTelemCallback(deviceid_t id, telemetrycode_t telemCode,
						 DataPoint<telemetry_t> data) {
	// first convert the telemetry code into a telemetry type
	auto telemTypeEntry = telemCodeToTypeMap.find(telemCode);
	if (telemTypeEntry != telemCodeToTypeMap.end()) {
		telemtype_t telemType = telemTypeEntry->second;
		std::lock_guard lock(telemetryCallbackMapMutex);
		// get callback (checking if it exists)
		auto entry = telemetryCallbackMap.find(std::make_pair(id, telemCode));
		if (entry != telemetryCallbackMap.end()) {
			// invoke all callbacks
			for (auto callbackEntry : entry->second) {
				callbackEntry.second(id, telemType, data);
			}
		}
	}
}

void handleTelemetryPacket(CANPacket& packet) {
	// extract necessary information
	deviceid_t id = getSenderDeviceGroupAndSerial(packet);
	telemetrycode_t telemCode = DecodeTelemetryType(&packet);
	telemetry_t telemData = DecodeTelemetryDataSigned(&packet);
	DataPoint<telemetry_t> data(telemData);

	// check if telemetry data is already in map
	if (mapHasKey(telemMapMutex, telemMap, id)) {
		// acquire read lock of entire map
		std::shared_lock mapLock(telemMapMutex);
		auto& pair = telemMap.at(id);
		std::shared_mutex& deviceMutex = *pair.first;
		auto& deviceMap = *pair.second;
		// acquire write lock of the map for this device
		std::unique_lock deviceLock(deviceMutex);
		// insert telemetry data
		deviceMap.insert_or_assign(telemCode, data);
	} else {
		// this device has no existing data, so insert a new device map
		auto mutexPtr = std::make_shared<std::shared_mutex>();
		auto deviceMapPtr =
			std::make_shared<std::unordered_map<telemetrycode_t, DataPoint<telemetry_t>>>();
		deviceMapPtr->emplace(telemCode, data);
		// acquire write lock of the entire map to insert a new device map
		std::unique_lock mapLock(telemMapMutex);
		telemMap.emplace(id, std::make_pair(mutexPtr, deviceMapPtr));
	}

	// now fire off callback, if it exists
	invokeTelemCallback(id, telemCode, data);
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

				case packettype_t::limit_alert:
					invokeTelemCallback(
						getSenderDeviceGroupAndSerial(packet),
						static_cast<telemetrycode_t>(telemtype_t::limit_switch),
						{packet.data[3]});
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

callbackid_t addDeviceTelemetryCallback(
	deviceid_t id, telemtype_t telemType,
	const std::function<void(deviceid_t, telemtype_t, robot::types::DataPoint<telemetry_t>)>&
		callback) {
	telemetrycode_t code = static_cast<telemetrycode_t>(telemType);
	auto key = std::make_pair(id, code);

	std::lock_guard lock(telemetryCallbackMapMutex);
	uint32_t callbackIDCode = nextCallbackID++;
	callbackid_t callbackID = {id, telemType, callbackIDCode};

	telemetryCallbackMap.insert({key, {}});
	telemetryCallbackMap.at(key).insert({callbackIDCode, callback});

	return callbackID;
}

void removeDeviceTelemetryCallback(callbackid_t id) {
	deviceid_t deviceID = std::get<0>(id);
	telemetrycode_t telemCode = static_cast<telemetrycode_t>(std::get<1>(id));
	uint32_t code = std::get<2>(id);
	auto key = std::make_pair(deviceID, telemCode);

	std::lock_guard lock(telemetryCallbackMapMutex);
	auto entry = telemetryCallbackMap.find(key);
	if (entry != telemetryCallbackMap.end()) {
		// remove the callback
		entry->second.erase(code);
		// if there are no callbacks left for this telemetry type + device, remove the map
		if (entry->second.empty()) {
			telemetryCallbackMap.erase(entry);
		}
	}
}

} // namespace can
