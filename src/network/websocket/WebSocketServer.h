#pragma once

#include "WebSocketProtocol.h"

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <thread>

#include <nlohmann/json.hpp>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

namespace net {
namespace websocket {

using nlohmann::json;
using websocketpp::connection_hdl;

typedef std::shared_ptr<websocketpp::config::core::message_type> message_t;

/**
 * @brief A WebSocket server class that only accepts a single client at a time
 * to each endpoint served by this server.
 * Users of this class will create endpoints by adding websocket protocols using addProtocol().
 * This server processes messages from the client and invokes message handlers,
 * which are defined by a websocket::WebSocketProtocol object.
 *
 * Messages recieved by this server are expected to be JSON-encoded.
 * Further requirements are dictated by the websocket::WebSocketProtocol documentation.
 * The entire json object will be passed to message handlers.
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
	 * @brief Destroy the server after calling stop()
	 *
	 */
	~SingleClientWSServer();

	// delete the assignment operator
	SingleClientWSServer& operator=(const SingleClientWSServer&) = delete;

	/**
	 * @brief Start the server.
	 *
	 * @return true iff the server was successfully started. May return false if the server is
	 * already running.
	 */
	bool start();

	/**
	 * @brief Stop the server. Disconnects from all clients and shuts down the server.
	 */
	void stop();

	/**
	 * @brief Register a protocol with this server. It will create an endpoint at a protocol
	 * path which clients can connect to. The protocol and message handlers are defined by the
	 * protocol object. If a protocol already exists at the given protocol path, do nothing.
	 *
	 * @param protocol The protocol object to add to the server.
	 * @return true iff no protocol existed with the given protocol path, and the given
	 * protocol was successfully added.
	 */
	bool addProtocol(std::unique_ptr<WebSocketProtocol> protocol);

	/**
	 * @brief Send a string to the client connected to the given protocol path, if there is
	 * one.
	 *
	 * @param protocolPath The protocol path which the client is connected to.
	 * @param str The message to send to the client.
	 */
	void sendRawString(const std::string& protocolPath, const std::string& str);

	/**
	 * @brief Serialize the JSON as a string and send it to the client connected
	 * to the given protocol path, if there is one.
	 *
	 * @param protocolPath The protocol path which the client is connected to.
	 * @param obj The JSON object to serialize and send to the client.
	 */
	void sendJSON(const std::string& protocolPath, const json& obj);

private:
	class ProtocolData {
	public:
		ProtocolData(std::unique_ptr<WebSocketProtocol> protocol);
		std::unique_ptr<WebSocketProtocol> protocol;
		std::optional<connection_hdl> client;
	};

	std::string serverName;
	uint16_t port;
	websocketpp::server<websocketpp::config::asio> server;
	bool isRunning;
	std::map<std::string, ProtocolData> protocolMap;
	std::thread serverThread;

	bool validate(connection_hdl hdl);
	void onOpen(connection_hdl hdl);
	void onClose(connection_hdl hdl);
	void onMessage(connection_hdl hdl, message_t message);
	void serverTask();
};
} // namespace websocket
} // namespace net
