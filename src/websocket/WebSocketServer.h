#pragma once

#include "../log.h"

#include <functional>
#include <map>
#include <optional>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

namespace websocket {

using websocketpp::connection_hdl;

typedef std::shared_ptr<websocketpp::config::core::message_type> message_t;

/**
 * @brief A WebSocket server class that only accepts a single client at a time.
 * This server processes messages from the client and invokes message handlers
 * depending on the "type" field of the JSON-encoded message.
 *
 * Messages recieved by this server are expected to be JSON-encoded, and need to have
 * a "type" field. The entire json object will be passed to messange handlers.
 */
class SingleClientWSServer {
public:
	/**
	 * @brief Construct a new server object, without starting it.
	 *
	 * @param serverName The name of this server. This is used only for logging purposes.
	 * @param port The local port to bind to. This is not validated until start() is called.
	 */
	SingleClientWSServer(const std::string& serverName, uint16_t port);

	/**
	 * @brief Start the server.
	 *
	 * @return true iff the server was successfully started. May return false if the server is
	 * already running.
	 */
	bool start();

	/**
	 * @brief Add a message handler for the given message type, if no handler already exists.
	 * If a handler already exists, this method no-ops.
	 *
	 * @param messageType The name of the message type this handler is designed for.
	 * @param callback The callback function that will accept the json object.
	 * @return true if no handler already existed. false if a handler already exists.
	 */
	bool addMessageHandler(const std::string& messageType,
						   const std::function<void(json)>& callback);

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

private:
	bool isRunning;
	uint16_t port;
	std::string serverName;
	websocketpp::server<websocketpp::config::asio> server;
	std::optional<connection_hdl> currConnection;
	std::map<std::string, std::function<void(json)>> callbackMap;

	bool validate(connection_hdl hdl);
	void onOpen(connection_hdl hdl);
	void onClose(connection_hdl hdl);
	void onMessage(connection_hdl hdl, message_t message);
};
} // namespace websocket
