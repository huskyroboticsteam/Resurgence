#include "WebSocketServer.h"

#include "../log.h"

#include <string>

#include <nlohmann/json.hpp>

namespace websocket {

using nlohmann::json;

static const std::string TYPE_KEY = "type";

SingleClientWSServer::SingleClientWSServer(const std::string& serverName, uint16_t port)
	: serverName(serverName), server(), port(port), currConnection(), isRunning(false) {
	server.init_asio();

	server.set_open_handler([&](connection_hdl hdl) { this->onOpen(hdl); });
	server.set_open_handler([&](connection_hdl hdl) { this->onClose(hdl); });
	server.set_validate_handler([&](connection_hdl hdl) { return this->validate(hdl); });
	server.set_message_handler(
		[&](connection_hdl hdl, message_t msg) { this->onMessage(hdl, msg); });
}

bool SingleClientWSServer::start() {
	if (isRunning) {
		return false;
	}

	try {
		server.listen(port);
		server.start_accept();
		server.run();
		isRunning = true;
		return true;
	} catch (const websocketpp::exception& e) {
		log(LOG_ERROR, "Server %s - An error occurred while starting: %s\n", serverName,
			e.what());
		return false;
	}
}

bool SingleClientWSServer::addMessageHandler(const std::string& messageType,
											 const std::function<void(json)>& callback) {
	if (!hasMessageHandler(messageType)) {
		callbackMap[messageType] = callback;
		return true;
	}
	return false;
}

bool SingleClientWSServer::hasMessageHandler(const std::string& messageType) const {
	return callbackMap.find(messageType) == callbackMap.end();
}

bool SingleClientWSServer::removeMessageHandler(const std::string& messageType) {
	return callbackMap.erase(messageType) != 0;
}

bool SingleClientWSServer::validate(connection_hdl hdl) {
	// reject connections if one client is already connected
	return !currConnection.has_value();
}

void SingleClientWSServer::onOpen(connection_hdl hdl) {
	auto conn = server.get_con_from_hdl(hdl);
	std::string client = conn->get_remote_endpoint();
	log(LOG_INFO, "Server %s - Connection opened from %s\n", serverName, client);
}

void SingleClientWSServer::onClose(connection_hdl hdl) {
	auto conn = server.get_con_from_hdl(hdl);
	std::string client = conn->get_remote_endpoint();
	log(LOG_INFO, "Server %s - Connection disconnected from %s\n", serverName, client);
}

void SingleClientWSServer::onMessage(connection_hdl hdl, message_t message) {
	std::string jsonStr = message->get_payload();
	json obj = json::parse(jsonStr);
	if (obj.contains(TYPE_KEY)) {
		std::string messageType = obj[TYPE_KEY];
		auto entry = callbackMap.find(messageType);
		if (entry != callbackMap.end()) {
			entry->second(obj);
		} else {
			log(LOG_WARN, "Server %s - Unrecognized message type: %s\n", serverName,
				messageType);
		}
	} else {
		log(LOG_WARN, "Server %s - Message from client missing \"type\" key: %s\n", serverName,
			jsonStr);
	}
}
} // namespace websocket
