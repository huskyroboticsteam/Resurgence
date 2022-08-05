#pragma once

#include <functional>
#include <map>
#include <string>

#include <nlohmann/json.hpp>

namespace net{
namespace websocket {

using nlohmann::json;

typedef std::function<void(const json&)> msghandler_t;
typedef std::function<bool(const json&)> validator_t;
typedef std::function<void()> connhandler_t;

/**
 * @brief Defines a protocol which will be served at an endpoint of a server.
 * This protocol defines several message types, and processes messages of those types,
 * dispatching events to message handlers wherever necessary.
 *
 * JSON messages must define a "type" key, which controls how messages are handled.
 * Additionally, JSON messages must (obviously) conform the expectations of the message
 * handlers. This can be verified at runtime by using the optional validation functionality.
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

	virtual ~WebSocketProtocol() = default;

	/**
	 * @brief Add a message handler for the given message type, if no handler already exists.
	 * Messages for this message type will not be validated.
	 * If a handler already exists, this method no-ops.
	 *
	 * @param messageType The name of the message type this handler is designed for.
	 * @param callback The callback function that will accept the json object.
	 * @return true if no handler already existed. false if a handler already exists.
	 */
	bool addMessageHandler(const std::string& messageType, const msghandler_t& callback);

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
	bool addMessageHandler(const std::string& messageType, const msghandler_t& callback,
						   const validator_t& validator);

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

	void addConnectionHandler(const connhandler_t& handler);

	void addDisconnectionHandler(const connhandler_t& handler);

	/**
	 * @brief Process the given JSON object that was sent to this protocol's endpoint.
	 * Generally, this shouldn't be used by client code.
	 *
	 * @param obj The JSON object to be processed by this protocol. It is expected to have a
	 * "type" key.
	 */
	void processMessage(const json& obj) const;

	/**
	 * @brief Invoke all connection handlers for this protocol.
	 * Generally, this shouldn't be used by client code.
	 */
	void clientConnected();

	/**
	 * @brief Invoke all disconnection handlers for this protocol.
	 * Generally, this shouldn't be used by client code.
	 */
	void clientDisconnected();

	/**
	 * @brief Get the protocol path of the endpoint this protocol is served on.
	 *
	 * @return The protocol path, of the form "/foo/bar".
	 */
	std::string getProtocolPath() const;

private:
	std::string protocolPath;
	std::map<std::string, msghandler_t> handlerMap;
	std::map<std::string, validator_t> validatorMap;
	std::vector<connhandler_t> connectionHandlers;
	std::vector<connhandler_t> disconnectionHandlers;
};

} // namespace websocket
} // namespace net
