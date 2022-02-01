#pragma once

#include <functional>
#include <map>
#include <string>

#include <nlohmann/json.hpp>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

namespace websocket {

using nlohmann::json;

/**
 * @brief Defines a protocol which will be served at an endpoint of a server.
 * This protocol defines several message types, and processes messages of those types,
 * dispatching events to message handlers wherever necessary.
 */
class WebSocketProtocol {
public:
	/**
	 * @brief Construct a new WebSocket protocol.
	 * This object doesn't itself "do" anything, but users should register this object with a
	 * server, which will then handle dispatching events.
	 *
	 * @param protocolPath The resource path that this protocol should be served on.
	 * This should be of the form "/foo/bar", for example.
	 */
	WebSocketProtocol(const std::string& protocolPath);

	/**
	 * @brief Add a message handler for the given message type, if no handler already exists.
	 * Messages for this message type will not be validated.
	 * If a handler already exists, this method no-ops.
	 *
	 * @param messageType The name of the message type this handler is designed for.
	 * @param callback The callback function that will accept the json object.
	 * @return true if no handler already existed. false if a handler already exists.
	 */
	bool addMessageHandler(const std::string& messageType,
						   const std::function<void(json)>& callback);

	/**
	 * @brief Add a message handler for the given message type, if no handler already exists.
	 * Messages for this message type will be validated using the given validator. The message
	 * handler is invoked iff the validator returns true for the json object.
	 * If a handler already exists, this method no-ops.
	 *
	 * @param messageType The name of the message type this handler is designed for.
	 * @param callback The callback function that will accept the json object.
	 * @param validator The validator function that validates the json object. Returns true iff
	 * the message is valid.
	 * @return true if no handler already existed. false if a handler already exists.
	 */
	bool addMessageHandler(const std::string& messageType,
						   const std::function<void(json)>& callback,
						   const std::function<bool(json)>& validator);

	/**
	 * @brief Remove the message handler for the given message type, if it exists.
	 *
	 * @param messageType The name of the message type to remove the handler for.
	 * @return true if a handler existed for this message type and was removed, false
	 * otherwise.
	 */
	bool removeMessageHandler(const std::string& messageType);

	/**
	 * @brief Check if the given message type has an associated message handler.
	 *
	 * @param messageType The name of the message to check for.
	 * @return true if a handler exists for this message type, false otherwise.
	 */
	bool hasMessageHandler(const std::string& messageType) const;

	/**
	 * @brief Process the given JSON object that was sent to this protocol's endpoint.
	 * Generally, this shouldn't be used by client code, but could be useful for testing/mocking.
	 * 
	 * @param obj The JSON object to be processed by this protocol.
	 */
	void processMessage(const json& obj) const;

	/**
	 * @brief Get the protocol path of the endpoint this protocol is served on.
	 * 
	 * @return The protocol path, of the form "/foo/bar".
	 */
	std::string getProtocolPath() const;

private:
	std::string protocolPath;
	std::map<std::string, std::function<void(json)>> handlerMap;
	std::map<std::string, std::function<bool(json)>> validatorMap;
};

} // namespace websocket
