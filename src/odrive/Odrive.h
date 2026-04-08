#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

struct Endpoint {
    uint16_t id;
    std::string type;
    bool readable;
    bool writable;
};

class Odrive {
public:
    bool loadDefaults();
    bool write(std::string endpoint, float value);
    float read(std::string endpoint);
    std::vector<std::string> search(std::string prefix);

private:
    std::unordered_map<std::string, Endpoint> endpointMap;
    std::unordered_map<std::string, float> valueMap;
};

