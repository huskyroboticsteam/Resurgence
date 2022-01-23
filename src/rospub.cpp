#include "rospub.h"

#include "Constants.h"

#include <array>
#include <map>
#include <set>
#include <string>
#include <thread>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace rospub {
namespace ws = websocketpp;

class PlanVizServer {
public:
	PlanVizServer();
	~PlanVizServer();
	void run(uint16_t port);
	void stop();
	bool running();
	void publish(const pose_t& pose, PointPub topic);
	void publish(const points_t& points, ArrayPub topic);

private:
	typedef ws::server<ws::config::asio> server;
	typedef std::set<ws::connection_hdl, std::owner_less<ws::connection_hdl>> conn_set;

	void onOpen(ws::connection_hdl handle);
	void onClose(ws::connection_hdl handle);
	void publish(const json& msg);

	bool _running;
	server _endpoint;
	conn_set _connections;
	std::thread _server_thread;
};

static PlanVizServer server;

void init() {
	if (!server.running()) {
		server.run(Constants::PLANVIZ_SERVER_PORT);
	}
}

void shutdown() {
	server.stop();
}

void publish(const pose_t& pose, PointPub topic) {
	server.publish(pose, topic);
}

void PlanVizServer::publish(const pose_t& pose, PointPub topic) {
	json message;
	message["topic"] = topic;
	message["x"] = pose(0);
	message["y"] = pose(1);
	message["theta"] = pose(2);
	this->publish(message);
}

void publish_array(const points_t& points, ArrayPub topic) {
	server.publish(points, topic);
}

void PlanVizServer::publish(const points_t& points, ArrayPub topic) {
	json message;
	message["topic"] = topic;
	message["points"] = json::array();
	for (point_t current : points) {
		auto obj = json::object();
		obj["x"] = current(0);
		obj["y"] = current(1);
		obj["theta"] = current(2);
		message["points"].push_back(obj);
	}
	this->publish(message);
}

void PlanVizServer::publish(const json& message) {
	std::string msg_str = message.dump();
	for (ws::connection_hdl handle : _connections) {
		_endpoint.send(handle, msg_str, ws::frame::opcode::text);
	}
}

PlanVizServer::PlanVizServer() {
	using ws::lib::placeholders::_1;
	_endpoint.init_asio();
	_endpoint.set_open_handler(ws::lib::bind(&PlanVizServer::onOpen, this, _1));
	_endpoint.set_close_handler(ws::lib::bind(&PlanVizServer::onClose, this, _1));
}

PlanVizServer::~PlanVizServer() {
	this->stop();
}

void PlanVizServer::run(uint16_t port) {
	if (!_running) {
		_endpoint.listen(port);
		_endpoint.start_accept();
		_server_thread = std::thread([this] { this->_endpoint.run(); });
		_running = true;
	}
}

void PlanVizServer::onOpen(ws::connection_hdl handle) {
	this->_connections.insert(handle);
}

void PlanVizServer::onClose(ws::connection_hdl handle) {
	this->_connections.erase(handle);
}

void PlanVizServer::stop() {
	if (_running) {
		_endpoint.stop();
		_server_thread.join();
		_running = false;
	}
}

bool PlanVizServer::running() {
	return _running;
}

} // namespace rospub
