#include "CAN.h"
#include "../world_interface/real_world_constants.h"
#include "../world_interface/world_interface.h"

#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <shared_mutex>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <utility>

#include <linux/can/raw.h>
#include <loguru.hpp>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

extern "C" {
#include <CANCommandIDs.h>
}

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
constexpr std::chrono::milliseconds ACK_TIMEOUT(50);
constexpr std::chrono::milliseconds READ_TIMEOUT(500);
// CAN26 11-bit ID layout: [priority:1][deviceUUID:7][peripheral:1][power:1][motor:1]
// Match on UUID field to filter for packets addressed to this device
constexpr uint32_t CAN_MASK = 0x3F8; // UUID field

// Heartbeats should come in every 500ms, have some leniency
constexpr auto HEARTBEAT_TIMEOUT = std::chrono::milliseconds(1000);

std::shared_mutex bufferMutex;
std::queue<CANPacket_t> buffer;
uint32_t buffer_size_max = 0;

// map each device seen to a watchdog
std::unordered_map<CANDeviceUUID_t, std::unique_ptr<util::Watchdog<>>> heartbeatWatchdogMap;
std::mutex heartbeatWatchdogMapMutex;

int can_fd;				// file descriptor of outbound can connection
std::mutex socketMutex; // protects can_fd

std::shared_ptr<util::PeriodicScheduler<>> ackScheduler;
std::unordered_map<
	std::pair<CANDeviceUUID_t, CANCommand_t>,
	util::PeriodicScheduler<>::eventid_t> ackMap;

// Holds read callbacks
std::shared_mutex directReadCallbackMutex;
std::unordered_map<
	std::pair<CANDeviceUUID_t, uint16_t>,
	std::function<void(CANMotorPacket_BLDC_DirectReadResult_Decoded_t)>> directReadCallbackMap;
std::shared_mutex directReadStatusMutex;
std::unordered_map<
	std::pair<CANDeviceUUID_t, uint16_t>,
	bool> directReadStatusMap;

// Endpoint JSONs
nlohmann::json pro_endpoints;
nlohmann::json s1_endpoints;

// not thread-safe wrt file descriptor
bool receivePacket(int fd, CANPacket_t& packet) {
	int ret;
	can_frame frame;
	ret = read(fd, &frame, sizeof(can_frame));
	if (ret >= 0) {
		// Parse 11-bit CAN ID into CANDevice_t + priority
		// [priority:1][deviceUUID:7][peripheral:1][power:1][motor:1]
		uint16_t canID = frame.can_id & 0x7FF; // extract the 11-bit CAN ID from frame
		uint16_t deviceBits = canID & 0x3FF;   // extract lower 10 bits (device info)
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

void handleAck(CANPacket_t& packet) {
	auto decoded = CANUniversalPacket_Acknowledge_Decode(&packet);
	if (decoded.sender.deviceUUID == 16) {
		// 0x10
		return;
	}
	if (decoded.failure) {
		LOG_F(WARNING, "Ack received from 0x%x: FAIL", decoded.sender.deviceUUID);
	} else {
		// LOG_F(INFO, "Ack received from 0x%x: ok", decoded.sender.deviceUUID);
	}

	auto key = std::make_pair(static_cast<uint8_t>(decoded.sender.deviceUUID), decoded.commandID);
	auto it = ackMap.find(key);
	if (it != ackMap.end()) {
		auto eventID = it->second;
		ackMap.erase(it);
		ackScheduler->removeEvent(eventID);
	}
}

void handleDirectRead(CANPacket_t& packet) {
	auto decoded = CANMotorPacket_BLDC_DirectReadResult_Decode(&packet);

	// Fire off callback, if it exists
	auto key = std::make_pair(static_cast<uint8_t>(decoded.sender.deviceUUID), decoded.endpointID);
	// Read access
	std::shared_lock mapReadLock(directReadCallbackMutex);
	auto it = directReadCallbackMap.find(key);
	// Unlock in case callback wants to modify the map?
	mapReadLock.unlock();
	if (it != directReadCallbackMap.end()) {
		it->second(decoded);
		std::unique_lock readStatusLock(directReadStatusMutex);
		if (auto it_status = directReadStatusMap.find(key); it_status != directReadStatusMap.end()) {
			directReadStatusMap.insert_or_assign(key, true);
		}
	} else {
		// LOG_F(INFO, "No callback associated with read 0x%x %d", decoded.sender.deviceUUID, decoded.endpointID);
	}
}

void handleEncoderEstimates(CANPacket_t& packet) {
	auto decoded = CANMotorPacket_BLDC_EncoderEstimates_Decode(&packet);
	CANDeviceUUID_t uuid = packet.senderUUID;
	// Convert position from revolutions to millidegrees
	int32_t positionMdeg = static_cast<int32_t>(decoded.position * Constants::MILLIDEGREES_PER_REV);

	if(auto it = robot::UUIDBoardMap.find(uuid); it != robot::UUIDBoardMap.end()) {
		robot::handleMotorEncoderEstimate(it->second, positionMdeg);
	}

	// telemetrycode_t telemCode = static_cast<telemetrycode_t>(telemtype_t::angle);
	// storeTelemetry(uuid, telemCode, robot::types::DataPoint<telemetry_t>(positionMdeg));
}

// Heartbeat monitoring handler: create watchdog on first heartbeat and feed it on subsequent heartbeats
void handleHeartbeatPacket(CANPacket_t& packet) {
	{
		std::lock_guard lock(heartbeatWatchdogMapMutex);
		CANDeviceUUID_t uuid = packet.senderUUID;
		auto it = heartbeatWatchdogMap.find(uuid);
		// if the watchdog expires, log a warning.
		if (it == heartbeatWatchdogMap.end()) {
			heartbeatWatchdogMap.emplace(
				uuid, 
				std::make_unique<util::Watchdog<>>(HEARTBEAT_TIMEOUT, [uuid]() {
					LOG_F(WARNING, "Heartbeat timeout for device 0x%x", uuid);
				})
			);
		} else {
			// feed the watchdog to reset timer
			it->second->feed();
		}
	}
}

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

		setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filters, sizeof(filters));
	} else {
		// disable reception on this socket.
		setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0);
	}

	return fd;
}

bool sendCANFrame(const canfd_frame& frame) {
	std::lock_guard lock(socketMutex);
	// note that frame is a canfd_frame but we're using sizeof(can_frame)
	// not sure why this is required to work
	bool success = write(can_fd, &frame, sizeof(struct can_frame)) == sizeof(struct can_frame);
	tcdrain(can_fd);
	return success;
}

void receiveThreadFn() {
	loguru::set_thread_name("CAN_Receive");
	CANPacket_t packet;
	// create dedicated CAN socket for reading
	int recvFD = createCANSocket(Constants::JETSON_DEVICE);
	if (recvFD < 0) {
		LOG_F(ERROR, "Unable to open CAN connection!");
		return;
	}

	while (true) {
		// no synchronization necessary, since this thread owns the FD
		bool received = receivePacket(recvFD, packet);
		if (received) {
			// Add packet to buffer
			// std::unique_lock lock(bufferMutex);
			buffer.push(packet);
			// if (packet.command == CAN_COMMAND_ID__BLDC_DIRECT_READ_RESULT) {
			// 	LOG_F(INFO, "Adding result packet for 0x%x to buffer", packet.senderUUID);
			// }
			// if (buffer.size() > buffer_size_max) {
			// 	LOG_F(INFO, "New max buffer size: %d", buffer_size_max);
			// 	buffer_size_max = buffer.size();
			// }
			// lock.unlock();
		} else {
			// we had a bus error, so sleep for a bit
			std::this_thread::sleep_for(READ_ERR_SLEEP);
		}
	}
}

void processThreadFn() {
	while (true) {
		std::shared_lock lock(bufferMutex);
		if (buffer.empty()) { continue; }

		lock.unlock();
		std::unique_lock write_lock(bufferMutex);

		// Double check required after releasing lock
		if (buffer.empty()) { continue; }
		CANPacket_t packet = buffer.front();
		buffer.pop();
		// Done with buffer, unlock to allow more reading
		write_lock.unlock();

		// dispatch on CAN26 command ID
		switch (packet.command) {
			case CAN_COMMAND_ID__E_STOP:
				LOG_F(WARNING, "Received E-Stop from UUID 0x%x", packet.senderUUID);
				break;

			case CAN_COMMAND_ID__ACKNOWLEDGE:
				handleAck(packet);
				break;

			case CAN_COMMAND_ID__HEARTBEAT:
				handleHeartbeatPacket(packet);
				break;

			case CAN_COMMAND_ID__LIMIT_SWITCH_ALERT:
				// handleLimitSwitchAlert(packet);
				break;

			case CAN_COMMAND_ID__BLDC_DIRECT_READ_RESULT:
				handleDirectRead(packet);
				break;

			case CAN_COMMAND_ID__BLDC_ENCODER_ESTIMATE:
				handleEncoderEstimates(packet);
				break;
			
			default:
				LOG_F(WARNING, "Unrecognized CAN command: 0x%x from UUID 0x%x",
						packet.command, packet.senderUUID);
				break;
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

	// Load Odrive endpoint jsons (files relative to build/)
	std::ifstream ifs_pro("../src/CAN/pro_endpoints.json");
	std::ifstream ifs_s1("../src/CAN/s1_endpoints.json");

	pro_endpoints = nlohmann::json::parse(ifs_pro)["endpoints"];
	s1_endpoints = nlohmann::json::parse(ifs_s1)["endpoints"];

	// start thread for recieving CAN packets
	std::thread receiveThread(receiveThreadFn);
	receiveThread.detach();

	// start thread for processing CAN packets
	std::thread processThread(processThreadFn);
	processThread.detach();

	std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void sendCANPacket(const CANPacket_t& packet) {
	CANPacket_t mutablePacket = packet; // to pass, we make a mutable copy
	mutablePacket.command = CAN_ACK(packet.command);
	CANDeviceUUID_t uuid = packet.device.deviceUUID;
	canfd_frame frame;
	std::memset(&frame, 0, sizeof(frame));
	frame.can_id = CANGetPacketHeader(&mutablePacket);
	frame.len = CANGetDlc(&mutablePacket);
	std::memcpy(frame.data, CANGetData(&mutablePacket), frame.len);

	if (packet.command & 0x80 && packet.command != CAN_ACK(CAN_COMMAND_ID__BLDC_DIRECT_READ)) {
		if (!ackScheduler) {
			ackScheduler = std::make_shared<util::PeriodicScheduler<>>("CAN_AckSched");
		}

		auto key = std::make_pair(static_cast<uint8_t>(packet.senderUUID), packet.command);
		auto it = ackMap.find(key);
		if (it != ackMap.end()) {
			LOG_F(WARNING, "0x%x already has an outgoing packet! Ignoring..", uuid);
			return;
		}

		auto eventID = ackScheduler->scheduleEvent(ACK_TIMEOUT, [=]() {
			LOG_F(ERROR, "0x%x ACK TIMED OUT, RESENDING", uuid);
			sendCANFrame(frame);
		});

		ackMap.insert_or_assign(key, eventID);
	}

	bool success = sendCANFrame(frame);

	if (!success) {
		LOG_F(ERROR, "Failed to send CAN packet to uuid=%x: %s", uuid,
			  std::strerror(errno));
	}
}

void printCANPacket(const CANPacket_t& packet) {
	CANPacket_t mutablePacket = packet; // same as sendCANPacket
	std::stringstream ss;
	ss << "CAN: ";
	ss << std::hex << (packet.senderUUID) << "->";
	ss << std::hex << (packet.device.deviceUUID);
	// ss << " domain" << std::hex << ((CANGetPacketHeader(&mutablePacket) & 0x0007));
	ss << " " << std::hex << static_cast<uint>(packet.command);
	ss << " [";
	for (int i = 1; i < CANGetDlc(&mutablePacket); i++) {
		ss << std::hex << static_cast<uint>(CANGetData(&mutablePacket)[i]) << " ";
	}
	ss << "]";

	LOG_F(INFO, ss.str().c_str());
}

void addDirectReadCallback(CANDevice_t device, uint16_t endpoint, const std::function<void(CANMotorPacket_BLDC_DirectReadResult_Decoded_t)>& callback) {
	auto key = std::make_pair(static_cast<uint8_t>(device.deviceUUID), endpoint);
	// Write access
	std::unique_lock mapLock(directReadCallbackMutex);
	if (auto it = directReadCallbackMap.find(key); it != directReadCallbackMap.end()) {
		// LOG_F(WARNING, "Callback already exists for 0x%x endpoint %d! Ignoring..", device.deviceUUID, endpoint);
		return;
	}

	std::thread timeout([device, endpoint, key]() {
		std::unique_lock statusLock(directReadStatusMutex);
		directReadStatusMap.emplace(key, false);
		statusLock.unlock();
		std::this_thread::sleep_for(READ_TIMEOUT);

		statusLock.lock();
		if (auto it = directReadStatusMap.find(key); it != directReadStatusMap.end()) {
			if (!it->second) {
				LOG_F(ERROR, "0x%x read of %d timed out! Removing callback...", device.deviceUUID, endpoint);
				std::unique_lock mapLock(directReadCallbackMutex);
				directReadCallbackMap.erase(key);
			}
			directReadStatusMap.erase(key);
		}
	});
	timeout.detach();

	directReadCallbackMap.emplace(key, callback);
}

void removeDirectReadCallback(CANDevice_t device, uint16_t endpoint) {
	auto key = std::make_pair(static_cast<uint8_t>(device.deviceUUID), endpoint);
	// Write access
	std::unique_lock mapLock(directReadCallbackMutex);
	if (auto it = directReadCallbackMap.find(key); it != directReadCallbackMap.end()) {
		directReadCallbackMap.erase(it);
	}
}

nlohmann::json getEndpoint(boardid_t boardid, std::string endpoint) {
	nlohmann::json endpoints;
	if (auto it = robot::proBoards.find(boardid); it != robot::proBoards.end()) {
		endpoints = pro_endpoints;
	} else {
		endpoints = s1_endpoints;
	}

	if (!endpoints.contains(endpoint)) {
		LOG_F(ERROR, "Request for endpoint %s does not exist!", endpoint.c_str());
		return nullptr;
	}
	return endpoints[endpoint];
}

} // namespace can
