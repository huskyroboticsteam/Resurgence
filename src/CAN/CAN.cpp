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

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

extern "C" {
#include <HindsightCAN/CANCommon.h>
}

using robot::types::DataPoint;

template <typename T1, typename T2>
struct std::hash<std::pair<T1, T2>> {
    std::size_t operator()(const std::pair<T1, T2>& pair) const {
        auto h1 = std::hash<T1>()(pair.first);
        auto h2 = std::hash<T2>()(pair.second);
        return (h1 << 6) + (h1 >> 2) + h2 + 0x9e3779b9;
    }
};

namespace can {
namespace {
constexpr std::chrono::milliseconds READ_ERR_SLEEP(100);
constexpr uint32_t CAN_MASK = (0 << 10) | (0b1111 << 6) | 0;

int can_fd;
std::mutex socketMutex;

std::shared_ptr<util::PeriodicScheduler<>> telemScheduler;
std::unordered_map<std::pair<deviceid_t, telemtype_t>, util::PeriodicScheduler<>::eventid_t>
    telemEventIDMap;

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
    {PACKET_TELEMETRY_LIM_SW_STATE, telemtype_t::limit_switch},
    {PACKET_TELEMETRY_ADC_RAW, telemtype_t::adc_raw},
    {PACKET_TELEMETRY_SENSOR1, telemtype_t::velocity},
    {PACKET_TELEMETRY_SENSOR2, telemtype_t::odrive_temp}
};

using devicemap_t =
    std::pair<std::shared_ptr<std::shared_mutex>,
              std::shared_ptr<std::unordered_map<telemetrycode_t, DataPoint<std::any>>>>;

std::unordered_map<deviceid_t, devicemap_t> telemMap;
std::shared_mutex telemMapMutex;

std::unordered_map<
    std::pair<deviceid_t, telemetrycode_t>,
    std::unordered_map<uint32_t, std::function<void(deviceid_t, telemtype_t,
                                                    robot::types::DataPoint<std::any>)>>>
    telemetryCallbackMap;
uint32_t nextCallbackID = 0;
std::mutex telemetryCallbackMapMutex;

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
        LOG_F(ERROR, "Failed to receive CAN packet: %s", std::strerror(errno));
        return false;
    }
}

template <typename K, typename V>
bool mapHasKey(std::shared_mutex& mutex, const std::unordered_map<K, V>& map, const K& key) {
    std::shared_lock lock(mutex);
    return map.find(key) != map.end();
}

void invokeTelemCallback(deviceid_t id, telemetrycode_t telemCode,
                         DataPoint<std::any> data) {
    auto telemTypeEntry = telemCodeToTypeMap.find(telemCode);
    if (telemTypeEntry != telemCodeToTypeMap.end()) {
        telemtype_t telemType = telemTypeEntry->second;
        std::lock_guard lock(telemetryCallbackMapMutex);
        auto entry = telemetryCallbackMap.find(std::make_pair(id, telemCode));
        if (entry != telemetryCallbackMap.end()) {
            for (auto callbackEntry : entry->second) {
                callbackEntry.second(id, telemType, data);
            }
        }
    }
}

void handleTelemetryPacket(CANPacket& packet) {
    deviceid_t id = getSenderDeviceGroupAndSerial(packet);
    uint8_t cmd_id = packet.id & 0x3F; // Extract cmd_id for ODrive
    if (id.first == devicegroup_t::motor && cmd_id == ID_ODRIVE_GET_ENCODER_ESTIMATES) {
        float pos, vel;
        GetODriveGetEncoderEstimatesFromPacket(&packet, &pos, &vel);
        DataPoint<std::any> posData(pos);
        DataPoint<std::any> velData(vel);
        telemetrycode_t posCode = PACKET_TELEMETRY_ANG_POSITION;
        telemetrycode_t velCode = PACKET_TELEMETRY_SENSOR1;

        if (mapHasKey(telemMapMutex, telemMap, id)) {
            std::shared_lock mapLock(telemMapMutex);
            auto& pair = telemMap.at(id);
            std::shared_mutex& deviceMutex = *pair.first;
            auto& deviceMap = *pair.second;
            std::unique_lock deviceLock(deviceMutex);
            deviceMap.insert_or_assign(posCode, posData);
            deviceMap.insert_or_assign(velCode, velData);
        } else {
            auto mutexPtr = std::make_shared<std::shared_mutex>();
            auto deviceMapPtr = std::make_shared<std::unordered_map<telemetrycode_t, DataPoint<std::any>>>();
            deviceMapPtr->emplace(posCode, posData);
            deviceMapPtr->emplace(velCode, velData);
            std::unique_lock mapLock(telemMapMutex);
            telemMap.emplace(id, std::make_pair(mutexPtr, deviceMapPtr));
        }
        invokeTelemCallback(id, posCode, posData);
        invokeTelemCallback(id, velCode, velData);
    } else if (id.first == devicegroup_t::motor && cmd_id == ID_ODRIVE_GET_TEMPERATURE) {
        float fet_temp, motor_temp;
        GetODriveGetTemperatureFromPacket(&packet, &fet_temp, &motor_temp);
        DataPoint<std::any> tempData(std::make_pair(fet_temp, motor_temp));
        telemetrycode_t telemCode = PACKET_TELEMETRY_SENSOR2;

        if (mapHasKey(telemMapMutex, telemMap, id)) {
            std::shared_lock mapLock(telemMapMutex);
            auto& pair = telemMap.at(id);
            std::shared_mutex& deviceMutex = *pair.first;
            auto& deviceMap = *pair.second;
            std::unique_lock deviceLock(deviceMutex);
            deviceMap.insert_or_assign(telemCode, tempData);
        } else {
            auto mutexPtr = std::make_shared<std::shared_mutex>();
            auto deviceMapPtr = std::make_shared<std::unordered_map<telemetrycode_t, DataPoint<std::any>>>();
            deviceMapPtr->emplace(telemCode, tempData);
            std::unique_lock mapLock(telemMapMutex);
            telemMap.emplace(id, std::make_pair(mutexPtr, deviceMapPtr));
        }
        invokeTelemCallback(id, telemCode, tempData);
    } else {
        telemetrycode_t telemCode = DecodeTelemetryType(&packet);
        telemetry_t telemData = DecodeTelemetryDataSigned(&packet);
        DataPoint<std::any> data(telemData);

        if (mapHasKey(telemMapMutex, telemMap, id)) {
            std::shared_lock mapLock(telemMapMutex);
            auto& pair = telemMap.at(id);
            std::shared_mutex& deviceMutex = *pair.first;
            auto& deviceMap = *pair.second;
            std::unique_lock deviceLock(deviceMutex);
            deviceMap.insert_or_assign(telemCode, data);
        } else {
            auto mutexPtr = std::make_shared<std::shared_mutex>();
            auto deviceMapPtr = std::make_shared<std::unordered_map<telemetrycode_t, DataPoint<std::any>>>();
            deviceMapPtr->emplace(telemCode, data);
            std::unique_lock mapLock(telemMapMutex);
            telemMap.emplace(id, std::make_pair(mutexPtr, deviceMapPtr));
        }
        invokeTelemCallback(id, telemCode, data);
    }
}

int createCANSocket(std::optional<can::deviceid_t> id) {
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
            LOG_F(ERROR, "Failed to get virtual CAN interface index: %s", std::strerror(errno));
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

    if (id) {
        int canID = ConstructCANID(0, static_cast<uint8_t>(id->first), id->second);
        can_filter filters[1];
        filters[0].can_id = canID;
        filters[0].can_mask = CAN_MASK;
        setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filters, sizeof(filters));
    } else {
        setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0);
    }

    return fd;
}

void receiveThreadFn() {
    loguru::set_thread_name("CAN_Receive");
    CANPacket packet;
    int recvFD = createCANSocket({{devicegroup_t::master, DEVICE_SERIAL_JETSON}});
    if (recvFD < 0) {
        LOG_F(ERROR, "Unable to open CAN connection!");
        return;
    }

    while (true) {
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
                        DataPoint<std::any>(packet.data[3]));
                    break;
                default:
                    LOG_F(WARNING, "Unrecognized CAN packet type: %x", packetType);
                    break;
            }
        } else {
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
    std::thread receiveThread(receiveThreadFn);
    receiveThread.detach();
}

void sendCANPacket(const CANPacket& packet) {
    canfd_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = packet.id;
    frame.len = packet.dlc;
    std::memcpy(frame.data, packet.data, packet.dlc);
    bool success;
    {
        std::lock_guard lock(socketMutex);
        success = write(can_fd, &frame, sizeof(struct can_frame)) == sizeof(struct can_frame);
        tcdrain(can_fd);
    }
    if (!success) {
        LOG_F(ERROR, "Failed to send CAN packet to group=%x, id=%x: %s",
              static_cast<int>(getDeviceGroup(packet)), getDeviceSerial(packet),
              std::strerror(errno));
    }
}

robot::types::DataPoint<std::any> getDeviceTelemetry(deviceid_t id, telemtype_t telemType) {
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

void pullDeviceTelemetry(deviceid_t id, telemtype_t telemType) {
    CANPacket p;
    auto groupCode = static_cast<uint8_t>(id.first);
    auto serial = static_cast<uint8_t>(id.second);
    auto telemCode = static_cast<uint8_t>(telemType);
    AssembleTelemetryPullPacket(&p, groupCode, serial, telemCode);
    sendCANPacket(p);
}

void scheduleTelemetryPull(deviceid_t id, telemtype_t telemType,
                           std::chrono::milliseconds period) {
    if (!telemScheduler) {
        telemScheduler = std::make_shared<util::PeriodicScheduler<>>("CAN_TelemPullSched");
    }
    auto mapKey = std::make_pair(id, telemType);
    auto it = telemEventIDMap.find(mapKey);
    if (it != telemEventIDMap.end()) {
        auto eventID = it->second;
        telemEventIDMap.erase(it);
        telemScheduler->removeEvent(eventID);
    }
    auto eventID =
        telemScheduler->scheduleEvent(period, [=]() { pullDeviceTelemetry(id, telemType); });
    telemEventIDMap.insert_or_assign(mapKey, eventID);
    pullDeviceTelemetry(id, telemType);
}

void unscheduleTelemetryPull(deviceid_t id, telemtype_t telemType) {
    if (!telemScheduler) {
        return;
    }
    auto mapKey = std::make_pair(id, telemType);
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
    deviceid_t id, telemtype_t telemType,
    const std::function<void(deviceid_t, telemtype_t, robot::types::DataPoint<std::any>)>& callback) {
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
        entry->second.erase(code);
        if (entry->second.empty()) {
            telemetryCallbackMap.erase(entry);
        }
    }
}
} // namespace can
