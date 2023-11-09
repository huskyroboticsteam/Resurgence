#include "WebSocketServer.h"

#include "../../Constants.h"
#include "../../utils/core.h"

#include <loguru.hpp>
#include <string>
namespace net {
namespace websocket {

using nlohmann::json;

SingleClientWSServer::ProtocolData::ProtocolData(std::unique_ptr<WebSocketProtocol> protocol)
	: protocol(std::move(protocol)) {}

SingleClientWSServer::SingleClientWSServer(const std::string& serverName, uint16_t port)
	: serverName(serverName), port(port), server(), isRunning(false), protocolMap(),
	  serverThread() {
	// disable websocket logging
	server.set_access_channels(websocketpp::log::alevel::none);
	server.set_error_channels(websocketpp::log::elevel::none);
	server.set_reuse_addr(true);
	server.init_asio();

	server.set_open_handler([&](connection_hdl hdl) { this->onOpen(hdl); });
	server.set_close_handler([&](connection_hdl hdl) { this->onClose(hdl); });
	server.set_validate_handler([&](connection_hdl hdl) { return this->validate(hdl); });
	server.set_message_handler(
		[&](connection_hdl hdl, message_t msg) { this->onMessage(hdl, msg); });
	server.set_pong_handler(
		[&](connection_hdl hdl, std::string payload) { this->onPong(hdl, payload); });
}

SingleClientWSServer::~SingleClientWSServer() {
	stop();
}

bool SingleClientWSServer::start() {
	if (isRunning) {
		return false;
	} else {
		isRunning = true;
		serverThread = std::thread([&]() { return this->serverTask(); });
		return true;
	}
}

void SingleClientWSServer::serverTask() {
	try {
		server.listen(port);
		server.start_accept();
		server.run();
	} catch (const websocketpp::exception& e) {
		LOG_F(ERROR, "Server=%s - An error occurred while starting: %s", serverName.c_str(),
			  e.what());
	}
}

void SingleClientWSServer::stop() {
	if (isRunning) {
		isRunning = false;
		server.stop_listening();
		for (auto& entry : protocolMap) {
			if (entry.second.client) {
				try {
					server.close(entry.second.client.value(),
								 websocketpp::close::status::going_away,
								 "Server shutting down");
				} catch (const websocketpp::exception& e) {
					LOG_F(ERROR, "Server=%s : An error occurred while shutting down: %s",
						  serverName.c_str(), e.what());
				}
				entry.second.client.reset();
			}
		}
		if (serverThread.joinable()) {
			serverThread.join();
		}
	}
}

bool SingleClientWSServer::addProtocol(std::unique_ptr<WebSocketProtocol> protocol) {
	std::string path = protocol->getProtocolPath();
	if (protocolMap.find(path) == protocolMap.end()) {
		protocolMap.emplace(path, std::move(protocol));
		return true;
	} else {
		return false;
	}
}

void SingleClientWSServer::sendRawString(const std::string& protocolPath,
										 const std::string& str) {
	auto entry = protocolMap.find(protocolPath);
	if (entry != protocolMap.end()) {
		auto& protocolData = entry->second;
		if (protocolData.client) {
			connection_hdl hdl = protocolData.client.value();
			auto conn = server.get_con_from_hdl(hdl);
			conn->send(str, websocketpp::frame::opcode::text);
		}
	} else {
		LOG_F(WARNING, "Server=%s : Can't send message to nonexistent endpoint: %s",
			  serverName.c_str(), protocolPath.c_str());
	}
}

void SingleClientWSServer::sendJSON(const std::string& protocolPath, const json& obj) {
	this->sendRawString(protocolPath, obj.dump());
}

bool SingleClientWSServer::validate(connection_hdl hdl) {
	auto conn = server.get_con_from_hdl(hdl);
	std::string path = conn->get_resource();
	auto entry = protocolMap.find(path);
	if (entry != protocolMap.end()) {
		if (!entry->second.client.has_value()) {
			return true;
		} else {
			auto existingConn = server.get_con_from_hdl(entry->second.client.value());
			LOG_F(INFO,
				"Server=%s, Endpoint=%s : Rejected connection from %s - A client is already "
				"connected: %s\n",
				serverName.c_str(), path.c_str(), conn->get_remote_endpoint().c_str(),
				existingConn->get_remote_endpoint().c_str());
			return false;
		}
	} else {
		LOG_F(INFO, "Server=%s : Rejected connection to unrecognized endpoint %s from %s",
			  serverName.c_str(), path.c_str(), conn->get_remote_endpoint().c_str());
		return false;
	}
}

void SingleClientWSServer::onOpen(connection_hdl hdl) {
	auto conn = server.get_con_from_hdl(hdl);
	std::string client = conn->get_remote_endpoint();
	std::string path = conn->get_resource();
	LOG_F(INFO, "Server=%s, Endpoint=%s : Connection opened from %s", serverName.c_str(),
		  path.c_str(), client.c_str());

	auto& protocolData = protocolMap.at(path);
	{
		std::lock_guard lock(protocolData.mutex);
		protocolData.client = hdl;
		const auto& heartbeatInfo = protocolData.protocol->heartbeatInfo;
		if (heartbeatInfo.has_value()) {
			auto eventID =
				pingScheduler.scheduleEvent(heartbeatInfo->first / 2, [this, path]() {
					auto& pd = this->protocolMap.at(path);
					std::lock_guard lock(pd.mutex);
					if (pd.client.has_value()) {
						LOG_F(2, "Ping!");
						server.ping(pd.client.value(), path);
					}
				});
			// util::Watchdog is non-copyable and non-movable, so we must create in-place
			// Since we want to create a member field of the pair in-place, it gets complicated
			// so we have to use piecewise_construct to allow us to separately initialize all
			// pair fields in-place
			protocolData.heartbeatInfo.emplace(std::piecewise_construct,
											   std::tuple<decltype(eventID)>{eventID},
											   util::pairToTuple(heartbeatInfo.value()));
		}

		protocolData.protocol->clientConnected();
	}
}

void SingleClientWSServer::onClose(connection_hdl hdl) {
	auto conn = server.get_con_from_hdl(hdl);
	std::string client = conn->get_remote_endpoint();
	std::string path = conn->get_resource();
	LOG_F(INFO, "Server=%s, Endpoint=%s : Connection disconnected from %s", serverName.c_str(),
		  path.c_str(), client.c_str());

	auto& protocolData = protocolMap.at(path);
	{
		std::lock_guard lock(protocolData.mutex);
		protocolData.client.reset();
		if (protocolData.heartbeatInfo.has_value()) {
			pingScheduler.removeEvent(protocolData.heartbeatInfo->first);
			protocolData.heartbeatInfo.reset();
		}
		protocolData.protocol->clientDisconnected();
	}
}

void SingleClientWSServer::onMessage(connection_hdl hdl, message_t message) {
	auto conn = server.get_con_from_hdl(hdl);
	std::string path = conn->get_resource();

	auto it = protocolMap.find(path);
	if (it != protocolMap.end()) {
		std::string jsonStr = message->get_payload();
		LOG_F(1, "Message on %s: %s", path.c_str(), jsonStr.c_str());
		json obj = json::parse(jsonStr);
		it->second.protocol->processMessage(obj);
	} else {
		LOG_F(WARNING, "Received message on unknown protocol path %s", path.c_str());
	}
}

void SingleClientWSServer::onPong(connection_hdl hdl, const std::string& payload) {
	LOG_F(2, "Pong from %s", payload.c_str());
	auto conn = server.get_con_from_hdl(hdl);

	auto it = protocolMap.find(payload);
	if (it != protocolMap.end()) {
		auto& pd = it->second;
		std::lock_guard lock(pd.mutex);
		if (pd.heartbeatInfo.has_value()) {
			pd.heartbeatInfo->second.feed();
		}
	} else {
		LOG_F(WARNING, "Received pong on unknown protocol path %s", payload.c_str());
	}
}
} // namespace websocket
} // namespace net
