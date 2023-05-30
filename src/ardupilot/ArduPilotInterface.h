#pragma once

#include "ArduPilotProtocol.h"

namespace ardupilot {
/* @brief Creates a new ArduPilotProtocol
 *
 * @param A websocket server on which to add the protocol
 */
void initArduPilotProtocol(net::websocket::SingleClientWSServer& websocketServer);
} // namespace ardupilot
