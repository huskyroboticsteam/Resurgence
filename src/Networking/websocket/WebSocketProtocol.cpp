#include "WebSocketProtocol.h"

#include "../../log.h"

namespace websocket {

static const std::string TYPE_KEY = "type";

WebSocketProtocol::WebSocketProtocol(const std::string& protocolPath)
	: protocolPath(protocolPath), handlerMap(), validatorMap() {}

bool WebSocketProtocol::addMessageHandler(const std::string& messageType,
										  const std::function<void(json)>& callback) {
	return this->addMessageHandler(messageType, callback, [](json) { return true; });
}

bool WebSocketProtocol::addMessageHandler(const std::string& messageType,
										  const std::function<void(json)>& callback,
										  const std::function<bool(json)>& validator) {
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

void WebSocketProtocol::processMessage(const json& obj) const {
	if (obj.contains(TYPE_KEY)) {
		std::string messageType = obj[TYPE_KEY];
		auto validatorEntry = validatorMap.find(messageType);
		if (validatorEntry != validatorMap.end()) {
			if (validatorEntry->second(obj)) {
				handlerMap.at(messageType)(obj);
			} else {
				log(LOG_WARN, "Endpoint=%s : Invalid message received of type=%s: %s\n",
					protocolPath.c_str(), messageType.c_str(), obj.dump().c_str());
			}
		} else {
			log(LOG_WARN, "Endpoint=%s : Unrecognized message type: %s\n",
				protocolPath.c_str(), messageType.c_str());
		}
	} else {
		log(LOG_WARN, "Endpoint=%s : Malformed message without type key: %s\n",
			protocolPath.c_str(), obj.dump().c_str());
	}
}

std::string WebSocketProtocol::getProtocolPath() const {
	return protocolPath;
}

} // namespace websocket