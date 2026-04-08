#include "ODriveProtocol.h"

#include "../Constants.h"
#include "../Globals.h"

#include "ODriveMessages.h"

using namespace robot::types;
using namespace std::chrono_literals;

using val_t = nlohmann::json::value_t;
using net::websocket::connhandler_t;
using net::websocket::msghandler_t;
using net::websocket::validator_t;
using robot::types::boardid_t;
using std::placeholders::_1;

