#pragma once

#include "../websocket/WebSocketProtocol.h"
#include "../../world_interface/world_interface.h"
#include <unordered_set>

namespace mc {
using json = nlohmann::json;

websocket::WebSocketProtocol initMissionControlProtocol();
std::unordered_set<CameraID> getOpenCameraStreams();

};
