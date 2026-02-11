#include "CAN.h"

#include "CANUtils.h"

#include <chrono>
#include <cstring>
#include <loguru.hpp>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <utility>
// temp for printing
#include <iostream>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

extern "C" {
// new
#include <CANPacket.h>
#include <CANCommandIDs.h>
#include <Packets/DecodeMotor.h>
#include <Packets/DecodePeripheral.h>
#include <Packets/DecodePower.h>
#include <Packets/DecodeUniversal.h>

// old
#include <HindsightCAN/CANCommon.h>
}

using robot::types::DataPoint;

// template specialization for hashing pairs
template <typename T1, typename T2>
struct std::hash<std::pair<T1, T2>> {
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
// CAN26 11-bit ID layout: [priority:1][deviceUUID:7][peripheral:1][power:1][motor:1]
// Match on UUID field to filter for packets addressed to this device
constexpr uint32_t CAN_MASK = 0x3F8; // UUID field

int can_fd;				// file descriptor of outbound can connection
std::mutex socketMutex; // protects can_fd

std::shared_ptr<util::PeriodicScheduler<>> telemScheduler;
std::unordered_map<std::pair<CANDeviceUUID_t, telemtype_t>, util::PeriodicScheduler<>::eventid_t>
	telemEventIDMap;

using telemetrycode_t = uint8_t;

const std::unordered_map<telemetrycode_t, telemtype_t> telemCodeToTypeMap = {
	// TODO: these definiations are from HindsightCAN's CANCommon.h
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
	{PACKET_TELEMETRY_LIM_SW_STATE, telemtype_t::limit_switch},
	{PACKET_TELEMETRY_ADC_RAW, telemtype_t::adc_raw}};

// the telemetry map will store telemetry code instead of telem enum
// this means unrecognized telemetry types won't cause UB
using devicemap_t =
	std::pair<std::shared_ptr<std::shared_mutex>,
			  std::shared_ptr<std::unordered_map<telemetrycode_t, DataPoint<telemetry_t>>>>;

// holds telemetry data for each device
std::unordered_map<CANDeviceUUID_t, devicemap_t> telemMap;
std::shared_mutex telemMapMutex;

std::unordered_map<
	std::pair<CANDeviceUUID_t, telemetrycode_t>,
	std::unordered_map<uint32_t, std::function<void(CANDeviceUUID_t, telemtype_t,
													robot::types::DataPoint<telemetry_t>)>>>
	telemetryCallbackMap;
uint32_t nextCallbackID = 0;
std::mutex telemetryCallbackMapMutex; // protects callbackMap and callbackID

// not thread-safe wrt file descriptor
bool receivePacket(int fd, CANPacket_t& packet) {
	int ret;
	can_frame frame;
	ret = read(fd, &frame, sizeof(can_frame));
	if (ret >= 0) {
		// Parse 11-bit CAN ID into CANDevice_t + priority
		// [priority:1][deviceUUID:7][peripheral:1][power:1][motor:1]
		uint16_t canID = frame.can_id & 0x7FF; // extract the 11-bit CAN ID from frame
		uint16_t deviceBits = canID & 0x3FF; // extract lower 10 bits (device info)
		std::memcpy(&packet.device, &deviceBits, sizeof(uint16_t));
		packet.priority = (canID & 0x400) ? CAN_PRIORITY_LOW : CAN_PRIORITY_HIGH;

		// Parse 8-byte CAN data
		// [command:1][senderUUID:1][contents:0-6]
		if (frame.can_dlc >= 2) {
			packet.command = frame.data[0];
			packet.senderUUID = frame.data[1];
			packet.contentsLength = frame.can_dlc - 2;
			std::memcpy(packet.contents, frame.data + 2, packet.contentsLength);
		} else {
			packet.command = 0;
			packet.senderUUID = 0;
			packet.contentsLength = 0;
		}
		return true;
	} else {
		LOG_F(ERROR, "Failed to receive CAN packet: %s", std::strerror(errno));
		return false;
	}
}

template <typename K, typename V>
bool mapHasKey(std::shared_mutex& mutex, const std::unordered_map<K, V>& map, const K& key) {
	std::shared_lock lock(mutex);
	return map.find(key) != map.end();
}

void invokeTelemCallback(CANDeviceUUID_t uuid, telemetrycode_t telemCode,
						 DataPoint<telemetry_t> data) {
	// first convert the telemetry code into a telemetry type
	auto telemTypeEntry = telemCodeToTypeMap.find(telemCode);
	if (telemTypeEntry != telemCodeToTypeMap.end()) {
		telemtype_t telemType = telemTypeEntry->second;
		std::lock_guard lock(telemetryCallbackMapMutex);
		// get callback (checking if it exists)
		auto entry = telemetryCallbackMap.find(std::make_pair(uuid, telemCode));
		if (entry != telemetryCallbackMap.end()) {
			// invoke all callbacks
			for (auto callbackEntry : entry->second) {
				callbackEntry.second(uuid, telemType, data);
			}
		}
	}
}


// Store telemetry data in the thread-safe map and fire callbacks
void storeTelemetry(CANDeviceUUID_t uuid, telemetrycode_t telemCode,
					DataPoint<telemetry_t> data) {
	// check if telemetry data is already in map
	if (mapHasKey(telemMapMutex, telemMap, uuid)) {
		// acquire read lock of entire map
		std::shared_lock mapLock(telemMapMutex);
		auto& pair = telemMap.at(uuid);
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
		telemMap.emplace(uuid, std::make_pair(mutexPtr, deviceMapPtr));
	}
	// now fire off callback, if it exists
	invokeTelemCallback(uuid, telemCode, data);
}

// CAN26 command-specific handlers (replace old handleTelemetryPacket)

void handleEncoderEstimates(CANPacket_t& packet) {
	auto decoded = CANMotorPacket_BLDC_EncoderEstimates_Decode(&packet);
	CANDeviceUUID_t uuid = packet.senderUUID;
	// Convert position from revolutions to millidegrees
	int32_t positionMdeg = static_cast<int32_t>(decoded.position * 360000.0f);
	telemetrycode_t telemCode = static_cast<telemetrycode_t>(telemtype_t::angle);
	storeTelemetry(uuid, telemCode, DataPoint<telemetry_t>(positionMdeg));
}

void handleLimitSwitchAlert(CANPacket_t& packet) {
	auto decoded = CANMotorPacket_LimitSwitchAlert_Decode(&packet);
	CANDeviceUUID_t uuid = packet.senderUUID;
	telemetrycode_t telemCode = static_cast<telemetrycode_t>(telemtype_t::limit_switch);
	storeTelemetry(uuid, telemCode,
				   DataPoint<telemetry_t>(static_cast<telemetry_t>(decoded.switchStatus)));
}

/* old - replaced by command-specific handlers above
void handleTelemetryPacket(CANPacket_t& packet) {
	CANDeviceUUID_t uuid = getDeviceFromPacket(packet);
	telemetrycode_t telemCode = DecodeTelemetryType(&packet);
	telemetry_t telemData = DecodeTelemetryDataSigned(&packet);
	DataPoint<telemetry_t> data(telemData);
	storeTelemetry(uuid, telemCode, data);
}
*/

// returns a file descriptor, or -1 on failure
int createCANSocket(std::optional<CANDevice_t> device) {
	int fd;
	if ((fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		LOG_F(ERROR, "Failed to initialize CAN bus: %s", std::strerror(errno));
		return -1;
	}

	struct ifreq ifr;
	std::strcpy(ifr.ifr_name, "can0");
	if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
		LOG_F(ERROR, "Failed to get hardware CAN interface index: %s", std::strerror(errno));
		std::strcpy(ifr.ifr_name, "vcan0");
		if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
			LOG_F(ERROR, "Failed to get virtual CAN interface index: %s",
				  std::strerror(errno));
			return -1;
		}
		LOG_F(INFO, "Found virtual CAN interface index.");
	}

	struct sockaddr_can addr;
	std::memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
		LOG_F(ERROR, "Error binding CAN socket: %s", std::strerror(errno));
		return -1;
	}

	// enable reception at the given device, if provided
	if (device) {
		// Build CAN ID from CANDevice_t using CAN26's packet header format
		CANPacket_t dummy = {};
		dummy.device = *device;
		dummy.priority = CAN_PRIORITY_LOW;
		uint16_t canID = CANGetPacketHeader(&dummy);
		can_filter filters[1];
		filters[0].can_id = canID;
		filters[0].can_mask = CAN_MASK;

		// for testing
		std::cout << "CAN packet: " << canID << std::endl;
		std::cout << "CAN uuid: " << (canID & CAN_MASK) << std::endl;
		std::cout << "Filters: " << filters[0].can_id << ", " << filters[0].can_mask << std::endl;
		
		setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filters, sizeof(filters));
	} else {
		// disable reception on this socket.
		setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0);
	}

	return fd;
}

void receiveThreadFn() {
	loguru::set_thread_name("CAN_Receive");
	CANPacket_t packet;
	// create dedicated CAN socket for reading
	CANDevice_t jetsonDevice = {0, 0, 0, CAN_UUID_JETSON};
	int recvFD = createCANSocket(jetsonDevice);
	if (recvFD < 0) {
		LOG_F(ERROR, "Unable to open CAN connection!");
		return;
	}

	while (true) {
		// no synchronization necessary, since this thread owns the FD
		bool received = receivePacket(recvFD, packet);
		if (received) {
			// dispatch on CAN26 command ID
			switch (packet.command) {
				case CAN_COMMAND_ID__BLDC_ENCODER_ESTIMATE:
					handleEncoderEstimates(packet);
					break;

				case CAN_COMMAND_ID__LIMIT_SWITCH_ALERT:
					handleLimitSwitchAlert(packet);
					break;

				case CAN_COMMAND_ID__HEARTBEAT:
					LOG_F(INFO, "Heartbeat from UUID 0x%x", packet.senderUUID);
					break;

				case CAN_COMMAND_ID__E_STOP:
					LOG_F(WARNING, "Received E-Stop from UUID 0x%x", packet.senderUUID);
					break;

				default:
					LOG_F(WARNING, "Unrecognized CAN command: 0x%x from UUID 0x%x",
						  packet.command, packet.senderUUID);
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
		LOG_F(ERROR, "Unable to open CAN connection!");
	}

	// start thread for recieving CAN packets
	std::thread receiveThread(receiveThreadFn);
	receiveThread.detach();
}

// new for CAN26
void sendCANPacket(const CANPacket_t& packet) {
	CANPacket_t mutablePacket = packet; // to pass, we make a mutable copy
	canfd_frame frame;
	std::memset(&frame, 0, sizeof(frame));
	frame.can_id = CANGetPacketHeader(&mutablePacket);
	frame.len = CANGetDlc(&mutablePacket);
	std::memcpy(frame.data, CANGetData(&mutablePacket), frame.len);
	bool success;
	{
		std::lock_guard lock(socketMutex);
		// note that frame is a canfd_frame but we're using sizeof(can_frame)
		// not sure why this is required to work
		success = write(can_fd, &frame, sizeof(struct can_frame)) == sizeof(struct can_frame);
		tcdrain(can_fd);
	}

	if (!success) {
		LOG_F(ERROR, "Failed to send CAN packet to uuid=%x: %s", getUUIDFromPacket(packet), std::strerror(errno));
	}
}

// old for backwards compatibility with HindsightCAN
void sendCANPacket(const CANPacket& packet) {
	canfd_frame frame;
	std::memset(&frame, 0, sizeof(frame));
	frame.can_id = packet.id;
	frame.len = packet.dlc;
	std::memcpy(frame.data, packet.data, packet.dlc);
	bool success;
	{
		std::lock_guard lock(socketMutex);
		// note that frame is a canfd_frame but we're using sizeof(can_frame)
		// not sure why this is required to work
		success = write(can_fd, &frame, sizeof(struct can_frame)) == sizeof(struct can_frame);
		tcdrain(can_fd);
	}

	if (!success) {
		LOG_F(ERROR, "Failed to send CAN packet to group=%x, id=%x: %s",
			  static_cast<int>(getDeviceGroup(packet)), getDeviceSerial(packet),
			  std::strerror(errno));
	}
}

// new for CAN26
void printCANPacket(const CANPacket_t& packet) {
	CANPacket_t mutablePacket = packet; // same as sendCANPacket
	std::stringstream ss;
	ss << "CAN: p" << std::hex << ((CANGetPacketHeader(&mutablePacket) >> 10) & 0x1);
	ss << " id" << std::hex << ((CANGetPacketHeader(&mutablePacket) & 0x03C0) >> 6);
	ss << " domain" << std::hex << ((CANGetPacketHeader(&mutablePacket) & 0x003F));	
	ss << " pid" << std::hex << static_cast<uint>(CANGetData(&mutablePacket)[0]);
	ss << " data:";
	for (int i = 1; i < CANGetDlc(&mutablePacket); i++) {
		ss << std::hex << static_cast<uint>(CANGetData(&mutablePacket)[i]) << " ";
	}

	LOG_F(INFO, ss.str().c_str());
}

/* old
void printCANPacket(const CANPacket& packet) {
  std::stringstream ss;
  ss << "CAN: p" << std::hex << ((packet.id >> 10) & 0x1);
  ss << " g" << std::hex << ((packet.id & 0x03C0) >> 6);
  ss << " s" << std::hex << ((packet.id & 0x003F));
  ss << " pid" << std::hex << static_cast<uint>(packet.data[0]);
  ss << " data:";
  for (int i = 1; i < packet.dlc; i++) {
    ss << std::hex << static_cast<uint>(packet.data[i]) << " ";
  }

  LOG_F(INFO, ss.str().c_str());
}
*/

robot::types::DataPoint<telemetry_t> getDeviceTelemetry(CANDeviceUUID_t uuid, telemtype_t telemType) {
	std::shared_lock mapLock(telemMapMutex); // acquire read lock
	// find entry for device in map
	auto entry = telemMap.find(uuid);
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

void pullDeviceTelemetry(CANDeviceUUID_t uuid, telemtype_t telemType) {
	static const CANDevice_t JETSON_DEVICE = {0, 0, 0, CAN_UUID_JETSON};
	CANDevice_t target = {0, 1, 0, uuid}; // assume motor for now?

	if (telemType == telemtype_t::angle) {
		// Request encoder position from the device
		CANPacket_t p = CANMotorPacket_BLDC_GetEncoderEstimates(JETSON_DEVICE, target, 0);
		sendCANPacket(p);
	}
}

void scheduleTelemetryPull(CANDeviceUUID_t uuid, telemtype_t telemType,
						   std::chrono::milliseconds period) {
	if (!telemScheduler) {
		telemScheduler = std::make_shared<util::PeriodicScheduler<>>("CAN_TelemPullSched");
	}

	auto mapKey = std::make_pair(uuid, telemType);
	auto it = telemEventIDMap.find(mapKey);
	if (it != telemEventIDMap.end()) {
		auto eventID = it->second;
		telemEventIDMap.erase(it);
		telemScheduler->removeEvent(eventID);
	}

	auto eventID =
		telemScheduler->scheduleEvent(period, [=]() { pullDeviceTelemetry(uuid, telemType); });
	telemEventIDMap.insert_or_assign(mapKey, eventID);

	// pull immediately since scheduling does not send right now
	pullDeviceTelemetry(uuid, telemType);
}

void unscheduleTelemetryPull(CANDeviceUUID_t uuid, telemtype_t telemType) {
	if (!telemScheduler) {
		return;
	}

	auto mapKey = std::make_pair(uuid, telemType);
	auto it = telemEventIDMap.find(mapKey);
	if (it != telemEventIDMap.end()) {
		auto eventID = it->second;
		telemEventIDMap.erase(it);
		telemScheduler->removeEvent(eventID);
	}
}

void unscheduleAllTelemetryPulls() {
	if (!telemScheduler) {
		return;
	}

	telemScheduler->clear();
	telemEventIDMap.clear();
}

callbackid_t addDeviceTelemetryCallback(
	CANDeviceUUID_t uuid, telemtype_t telemType,
	const std::function<void(CANDeviceUUID_t, telemtype_t, robot::types::DataPoint<telemetry_t>)>&
		callback) {
	telemetrycode_t code = static_cast<telemetrycode_t>(telemType);
	auto key = std::make_pair(uuid, code);

	std::lock_guard lock(telemetryCallbackMapMutex);
	uint32_t callbackIDCode = nextCallbackID++;
	callbackid_t callbackID = {uuid, telemType, callbackIDCode};

	telemetryCallbackMap.insert({key, {}});
	telemetryCallbackMap.at(key).insert({callbackIDCode, callback});

	return callbackID;
}

void removeDeviceTelemetryCallback(callbackid_t id) {
	CANDeviceUUID_t uuid = std::get<0>(id);
	telemetrycode_t telemCode = static_cast<telemetrycode_t>(std::get<1>(id));
	uint32_t code = std::get<2>(id);
	auto key = std::make_pair(uuid, telemCode);

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
