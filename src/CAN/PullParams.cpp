#include "CAN.h"
#include "CANBoard.h"
#include "../world_interface/real_world_constants.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <set>

const std::set<const char*> endpoints({
    "fw_version_major",
    "fw_version_minor",
    "fw_version_revision",
    "axis0.controller.config.vel_limit"
});

int main() {
    can::initCAN();

    std::string dir = "../odrive-config/";

    if (!std::filesystem::exists(dir)) {
        std::filesystem::create_directory(dir);
    }

    std::unordered_map<robot::types::boardid_t, std::shared_ptr<can::CANBoard>> board_ptrs;

    for (const auto& [board, device] : robot::boardDeviceMap) {
        if (!device.motorDomain || board == robot::types::boardid_t::hand) { continue; }
        std::shared_ptr<can::CANBoard> ptr = std::make_shared<can::CANBoard>(board, device);

        std::string name = util::to_string(board);
        std::string file = dir + name + ".json";

        std::ifstream rfs(file);

        nlohmann::json prev;
        // If path exists, create a temp file to dump into for now, otherwise just write directly
        if (rfs.is_open()) {
            file += '~';
            prev = nlohmann::json::parse(rfs);
        }
        std::ofstream wfs(file);

        // Go through each endpoint
        nlohmann::json obj(nlohmann::json::value_t::object);
        for (const char* endpoint : endpoints) {
            if (nlohmann::json json = can::getEndpoint(board, endpoint); json != nullptr) {
                can::endpointid_t endpoint_id = json["id"];
                bool finished = false;
                can::addDirectReadCallback(device, endpoint_id, [&, name=name](auto p, std::unique_lock<std::shared_mutex> lock) {
                    if (!lock.mutex()) {
                        finished = true;
                        return;
                    }

                    std::string value;
                    std::string type = json["type"];
                    if (type == "uint32") {
                        value = p.value_uint32;
                    } else if (type == "int32") {
                        value = p.value_int32;
                    } else if (type == "uint16") {
                        value = p.value_uint16;
                    } else if (type == "uint8") {
                        value = p.value_uint8;
                    } else if (type == "float") {
                        value = p.value_float;
                    } else if (type == "bool") {
                        value = p.value_bool;
                    }

                    if (prev != nullptr && prev[endpoint] != nullptr) {
                        if (prev[endpoint] != value) {
                            std::cout << name << " " << endpoint << ": " << "prev=" << prev[endpoint] << ",recv=" << value;

                            std::string in;
                            while (true) {
                                std::cout << "({p}rev/{r}ecv): ";
                                std::getline(std::cin, in);

                                if (in == "p" || in == "prev") {
                                    obj[endpoint] = prev[endpoint];
                                    break;
                                } else if (in == "r" || in == "recv") {
                                    obj[endpoint] = value;
                                    break;
                                } else {
                                    std::cerr << "unrecognized" << std::endl;
                                    continue;
                                }
                            }
                        } else {
                            obj[endpoint] = prev[endpoint];
                        }
                    } else {
                        obj[endpoint] = value;
                    }

                    // We only need this once, remove after we get a response
                    can::removeDirectReadCallback(ptr->getDevice(), endpoint_id, std::move(lock));
                    finished = true;
                }, true);

                ptr->read(endpoint_id);

                while (!finished) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
        }

        wfs << std::setw(4) << obj;
        wfs.close();

        if (rfs.is_open()) {
            std::filesystem::copy(file, file.substr(0, file.size() - 1), std::filesystem::copy_options::overwrite_existing);
            std::filesystem::remove(file);
        }
        rfs.close();
    }
}