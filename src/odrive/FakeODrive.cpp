#include "../Constants.h"
#include "../utils/json.h"
#include "../CAN/CANMotor.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "loguru.hpp"

using namespace std::chrono_literals;
using json = nlohmann::json;

class ODriveCommandHandler {
private:
    json endpointsConfig;
	json configScript;
    
    bool isValidEndpoint(const std::string& endpoint);
    bool isWritableEndpoint(const std::string& endpoint);

public:
    ODriveCommandHandler(const std::string& endpointsPath);
    void loadConfigScript(const std::string& configPath);
	void applyConfigScript(CANDevice_t device);
};

ODriveCommandHandler::ODriveCommandHandler(const std::string& endpointsPath) {
    std::ifstream file(endpointsPath);
    if (!file.is_open()) {
        LOG_F(ERROR, "Failed to open endpoints config: %s", endpointsPath.c_str());
        return;
    }
    file >> endpointsConfig;
}

void ODriveCommandHandler::loadConfigScript(const std::string& configPath) {
    std::ifstream file(configPath);
    if (!file.is_open()) {
        LOG_F(ERROR, "Failed to open config script: %s", configPath.c_str());
        return;
    }
    file >> configScript;
    LOG_F(INFO, "Loaded config script with %zu entries", configScript.size());
}

void ODriveCommandHandler::applyConfigScript(CANDevice_t device) {
    if (configScript.empty()) {
        LOG_F(WARNING, "No config script loaded");
        return;
    }

    if (!endpointsConfig.contains("endpoints") || !endpointsConfig["endpoints"].is_object()) {
        LOG_F(ERROR, "Invalid endpoints config: missing 'endpoints' object");
        return;
    }

    const json& endpoints = endpointsConfig["endpoints"];
    
    for (const auto& entry : configScript) {
        if (!entry.contains("endpoint") || !entry.contains("value")) {
            LOG_F(ERROR, "Invalid config entry: missing endpoint or value");
            continue;
        }
        
        std::string endpoint_name = entry["endpoint"];

        if (!isValidEndpoint(endpoint_name)) {
            LOG_F(ERROR, "Invalid endpoint in config: %s", endpoint_name.c_str());
            continue;
        }

        const json& endpointInfo = endpoints[endpoint_name];
        uint16_t id = endpointInfo["id"].get<uint16_t>();
        
        if (!isWritableEndpoint(endpoint_name)) {
            LOG_F(ERROR, "No write access for this endpoint: %s", endpoint_name.c_str());
            continue;
        }

        uint32_t value = 0;
        const std::string type = endpointInfo.value("type", "uint32");
        if (type == "float") {
            if (!entry["value"].is_number()) {
                LOG_F(ERROR, "Endpoint %s expects float value", endpoint_name.c_str());
                continue;
            }
            float floatVal = entry["value"].get<float>();
            std::memcpy(&value, &floatVal, sizeof(floatVal));
        } else if (type == "bool") {
            if (!entry["value"].is_boolean()) {
                LOG_F(ERROR, "Endpoint %s expects bool value", endpoint_name.c_str());
                continue;
            }
            value = entry["value"].get<bool>() ? 1u : 0u;
        } else {
            if (!entry["value"].is_number_integer() && !entry["value"].is_number_unsigned()) {
                LOG_F(ERROR, "Endpoint %s expects integer value", endpoint_name.c_str());
                continue;
            }
            value = entry["value"].get<uint32_t>();
        }

        can::motor::write(device, id, value);
        std::cout << "Applying config: " << endpoint_name << " value: " << value << std::endl;
        can::motor::read(device, id);
        std::cout << "Applied config: " << endpoint_name << " value: " << value << std::endl;
    }
    
    LOG_F(INFO, "Config script applied successfully");
}

bool ODriveCommandHandler::isValidEndpoint(const std::string& endpoint) {
    return endpointsConfig.contains("endpoints") && endpointsConfig["endpoints"].contains(endpoint);
}

bool ODriveCommandHandler::isWritableEndpoint(const std::string& endpoint) {
    return endpointsConfig["endpoints"][endpoint]["access"] == "rw" ||
		   endpointsConfig["endpoints"][endpoint]["access"] == "w";
}

int main() {
    can::initCAN();

    ODriveCommandHandler handler("src/odrive/odrive_endpoints.json");
    handler.loadConfigScript("src/odrive/config.json");

	CANDevice_t device = {0, 1, 0, CAN_UUID_BLDC_BASE};
	handler.applyConfigScript(device);
    return 0;
}