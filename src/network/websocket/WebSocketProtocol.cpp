#include "WebSocketProtocol.h"

#include "../../Constants.h"

#include <loguru.hpp>
namespace net {
namespace websocket {

static const std::string TYPE_KEY = "type";

WebSocketProtocol::WebSocketProtocol(const std::string& protocolPath)
	: protocolPath(protocolPath), handlerMap(), validatorMap() {}

bool WebSocketProtocol::addMessageHandler(const std::string& messageType,
										  const msghandler_t& callback) {
	return this->addMessageHandler(messageType, callback, [](const json&) { return true; });
}

bool WebSocketProtocol::addMessageHandler(const std::string& messageType,
										  const msghandler_t& callback,
										  const validator_t& validator) {
	if (!hasMessageHandler(messageType)) {
		handlerMap[messageType] = callback;
		validatorMap[messageType] = validator;
		return true;
	}
	return false;
}

bool WebSocketProtocol::hasMessageHandler(const std::string& messageType) const {
	return handlerMap.find(messageType) != handlerMap.end();
}

bool WebSocketProtocol::removeMessageHandler(const std::string& messageType) {
	if (handlerMap.erase(messageType) != 0) {
		validatorMap.erase(messageType);
		return true;
	}
	return false;
}

void WebSocketProtocol::addConnectionHandler(const connhandler_t& handler) {
	connectionHandlers.push_back(handler);
}

void WebSocketProtocol::addDisconnectionHandler(const connhandler_t& handler) {
	disconnectionHandlers.push_back(handler);
}

void WebSocketProtocol::setHeartbeatTimedOutHandler(std::chrono::milliseconds timeout,
													const heartbeattimeouthandler_t& handler) {
	heartbeatInfo = {timeout, handler};
}

void WebSocketProtocol::clientConnected() {
	for (const auto& f : connectionHandlers) {
		f();
	}
}

void WebSocketProtocol::clientDisconnected() {
	for (const auto& f : disconnectionHandlers) {
		f();
	}
}

void WebSocketProtocol::heartbeatTimedOut() {
	if (heartbeatInfo.has_value()) {
		heartbeatInfo->second();
	}
}

void WebSocketProtocol::processMessage(const json& obj) const {
	if (obj.contains(TYPE_KEY)) {
		std::string messageType = obj[TYPE_KEY];
		auto validatorEntry = validatorMap.find(messageType);
		if (validatorEntry != validatorMap.end()) {
			if (validatorEntry->second(obj)) {
				if (protocolPath == Constants::Network::MC_PROTOCOL_NAME) {
					LOG_F(INFO, "MC->R: %s", obj.dump().c_str());
				}
				handlerMap.at(messageType)(obj);
			} else {
				LOG_F(WARNING, "Endpoint=%s : Invalid message received of type=%s: %s",
					  protocolPath.c_str(), messageType.c_str(), obj.dump().c_str());
			}
		} else {
			LOG_F(WARNING, "Endpoint=%s : Unrecognized message type: %s", protocolPath.c_str(),
				  messageType.c_str());
		}
	} else {
		LOG_F(WARNING, "Endpoint=%s : Malformed message without type key: %s",
			  protocolPath.c_str(), obj.dump().c_str());
	}
}

std::string WebSocketProtocol::getProtocolPath() const {
	return protocolPath;
}

} // namespace websocket
} // namespace net
