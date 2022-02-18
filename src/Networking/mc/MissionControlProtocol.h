#pragma once

#include "../websocket/WebSocketProtocol.h"
#include "../../world_interface/world_interface.h"
#include <unordered_set>

namespace mc {
using json = nlohmann::json;

/**
   @brief Initialize the mission control protocol.

   @return A websocket::WebSocketProtocol object representing the protocol, which can be passed
   to SingleClientWSServer::addProtocol(), and contains the appropriate message
   handlers/validators.
 */
websocket::WebSocketProtocol initMissionControlProtocol();
/**
   @brief Get the set of open camera streams as a set of camera IDs.
 */
std::unordered_set<CameraID> getOpenCameraStreams();

};
