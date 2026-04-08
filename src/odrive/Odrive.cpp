#include "Odrive.h"

#include <algorithm>
#include <fstream>
#include <limits>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace {
void addEndpoint(std::unordered_map<std::string, Endpoint>& endpointMap,
                 std::unordered_map<std::string, float>& valueMap,
                 const std::string& name,
                 const std::string& type,
                 uint16_t id,
                 bool readable,
                 bool writable) {
    Endpoint endpoint;
    endpoint.id = id;
    endpoint.type = type;
    endpoint.readable = readable;
    endpoint.writable = writable;
    endpointMap.insert_or_assign(name, std::move(endpoint));
    valueMap.insert_or_assign(name, 0.0f);
}
} // namespace

bool Odrive::loadDefaults() {
    endpointMap.clear();
    valueMap.clear();

    std::ifstream f("src/odrive/odrive_endpoints.json");
    if (f.is_open()) {
        try {
            json data;
            f >> data;
            if (data.contains("endpoints") && data["endpoints"].is_object()) {
                for (const auto& it : data["endpoints"].items()) {
                    const std::string& name = it.key();
                    const json& entry = it.value();
                    uint16_t id = entry.value("id", 0);
                    std::string type = entry.value("type", "");
                    std::string access = entry.value("access", "");
                    bool readable = access.find('r') != std::string::npos;
                    bool writable = access.find('w') != std::string::npos;
                    addEndpoint(endpointMap, valueMap, name, type, id, readable, writable);
                }
            }
        } catch (const json::parse_error&) {
        }
    }

    if (endpointMap.empty()) {
        // if loading from json file fails, add these endpoints instead:
        addEndpoint(endpointMap, valueMap, "vbus_voltage", "float", 1, true, false);
        addEndpoint(endpointMap, valueMap, "ibus", "float", 2, true, false);
        addEndpoint(endpointMap, valueMap, "ibus_report_filter_k", "float", 3, true, true);
        addEndpoint(endpointMap, valueMap, "axis0.controller.input_vel", "float", 4, true, true);
        addEndpoint(endpointMap, valueMap, "axis0.encoder.vel_estimate", "float", 5, true, false);
    }

    return !endpointMap.empty();
}

bool Odrive::write(std::string endpoint, float value) {
    auto it = endpointMap.find(endpoint);
    if (it == endpointMap.end() || !it->second.writable) {
        return false;
    }

    valueMap.insert_or_assign(std::move(endpoint), value);
    return true;
}

float Odrive::read(std::string endpoint) {
    auto it = endpointMap.find(endpoint);
    if (it == endpointMap.end() || !it->second.readable) {
        return std::numeric_limits<float>::quiet_NaN();
    }

    auto valueIt = valueMap.find(endpoint);
    if (valueIt != valueMap.end()) {
        return valueIt->second;
    }

    return 0.0f;
}

std::vector<std::string> Odrive::search(std::string prefix) {
    std::vector<std::string> matches;
    for (const auto& kv : endpointMap) {
        if (kv.first.rfind(prefix, 0) == 0) {
            matches.push_back(kv.first);
        }
    }
    std::sort(matches.begin(), matches.end());
    return matches;
}