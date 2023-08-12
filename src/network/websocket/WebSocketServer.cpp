#include "WebSocketServer.h"

#include "../../Constants.h"
#include "../../log.h"

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
	server.set_pong_timeout_handler(
		[&](connection_hdl hdl, std::string payload) { this->onPongTimeout(hdl, payload); });
	server.set_pong_handler([](connection_hdl, std::string s) { log(LOG_INFO, "Pong from %s\n", s.c_str());});
	server.set_pong_timeout(1000);
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
		log(LOG_ERROR, "Server=%s - An error occurred while starting: %s\n",
			serverName.c_str(), e.what());
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
					log(LOG_ERROR, "Server=%s : An error occurred while shutting down: %s",
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
		const auto& pongInfo = protocolMap.at(path).protocol->pongInfo;
		if (pongInfo.has_value()) {
			auto eventID = pingScheduler.scheduleEvent(pongInfo->first / 2, [this, path]() {
				const auto& pd = this->protocolMap.at(path);
				if (pd.client.has_value()) {
					log(LOG_INFO, "Ping!\n");
					server.ping(pd.client.value(), path);
				}
			});
			protocolMap.at(path).pingEventID = eventID;
		}
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
		log(LOG_WARN, "Server=%s : Can't send message to nonexistent endpoint: %s\n",
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
			log(LOG_INFO,
				"Server=%s, Endpoint=%s : Rejected connection from %s - A client is already "
				"connected!\n",
				serverName.c_str(), path.c_str(), conn->get_remote_endpoint().c_str());
			return false;
		}
	} else {
		log(LOG_INFO, "Server=%s : Rejected connection to unrecognized endpoint %s from %s\n",
			serverName.c_str(), path.c_str(), conn->get_remote_endpoint().c_str());
		return false;
	}
}

void SingleClientWSServer::onOpen(connection_hdl hdl) {
	auto conn = server.get_con_from_hdl(hdl);
	std::string client = conn->get_remote_endpoint();
	std::string path = conn->get_resource();
	log(LOG_INFO, "Server=%s, Endpoint=%s : Connection opened from %s\n", serverName.c_str(),
		path.c_str(), client.c_str());

	auto& protocolData = protocolMap.at(path);
	protocolData.client = hdl;
	protocolData.protocol->clientConnected();
}

void SingleClientWSServer::onClose(connection_hdl hdl) {
	auto conn = server.get_con_from_hdl(hdl);
	std::string client = conn->get_remote_endpoint();
	std::string path = conn->get_resource();
	log(LOG_INFO, "Server=%s, Endpoint=%s : Connection disconnected from %s\n",
		serverName.c_str(), path.c_str(), client.c_str());

	auto& protocolData = protocolMap.at(path);
	protocolData.client.reset();
	if (protocolData.pingEventID.has_value()) {
		pingScheduler.removeEvent(protocolData.pingEventID.value());
		protocolData.pingEventID.reset();
	}
	protocolData.protocol->clientDisconnected();
}

void SingleClientWSServer::onMessage(connection_hdl hdl, message_t message) {
	auto conn = server.get_con_from_hdl(hdl);
	std::string path = conn->get_resource();

	assert(protocolMap.find(path) != protocolMap.end());

	std::string jsonStr = message->get_payload();
	log(LOG_TRACE, "Message on %s: %s\n", path.c_str(), jsonStr.c_str());
	json obj = json::parse(jsonStr);
	protocolMap.at(path).protocol->processMessage(obj);
}

void SingleClientWSServer::onPongTimeout(connection_hdl hdl, const std::string& payload) {
	auto conn = server.get_con_from_hdl(hdl);

	assert(protocolMap.find(payload) != protocolMap.end());

	log(LOG_ERROR, "Pong timeout on %s\n", payload.c_str());
	auto& pongInfo = protocolMap.at(payload).protocol->pongInfo;
	if (pongInfo.has_value()) {
		pongInfo->second();
	}
}
} // namespace websocket
} // namespace net
