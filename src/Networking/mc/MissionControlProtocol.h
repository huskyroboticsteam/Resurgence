#pragma once

#include "../websocket/WebSocketProtocol.h"

namespace proto {
using json = nlohmann::json;

websocket::WebSocketProtocol initMissionControlProtocol();

};
