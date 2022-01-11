#include "Networking/json.hpp"
#include "rospub.h"
#include "simulator/graphics.h"
#include "simulator/utils.h"

#include <iostream>

#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>

namespace ws = websocketpp;

using ws::lib::bind;
using ws::lib::placeholders::_1;
using ws::lib::placeholders::_2;
using json = nlohmann::json;

static double get_double_or_nan(json value) {
	if (value.is_null() || !value.is_number_float()) {
		return NAN;
	} else {
		return value.get<double>();
	}
}

class PlanViz {
public:
	PlanViz() : viz_window("Planning visualization") {
		endpoint.init_asio();
		endpoint.set_message_handler(bind(&PlanViz::on_message, this, _1, _2));
		viz_window.display();
	}
	bool run(const std::string& uri) {
		ws::lib::error_code ec;
		client::connection_ptr con = endpoint.get_connection(uri, ec);
		if (ec) {
			std::cerr << "Failed to connect: " << ec.message() << std::endl;
			return false;
		}
		endpoint.connect(con);
		endpoint.run();
		return true;
	}

private:
	MyWindow viz_window;
	typedef ws::client<ws::config::asio_client> client;
	typedef ws::config::asio_client::message_type::ptr message_ptr;
	client endpoint;

	void curr_pose_callback(const pose_t& pose) {
		viz_window.drawRobot(toTransform(pose), sf::Color::Black);
	}

	void drive_target_callback(const pose_t& target) {
		viz_window.drawRobot(toTransform(target), sf::Color::Blue);
	}

	void plan_target_callback(const pose_t& target) {
		viz_window.drawRobot(toTransform(target), sf::Color::Green);
	}

	void lidar_callback(const points_t& lidar) {
		viz_window.drawPoints(lidar, sf::Color::Red, 3);
	}

	void landmarks_callback(const points_t& landmarks) {
		viz_window.drawPoints(landmarks, sf::Color::Blue, 5);
	}

	void plan_callback(const pose_t& point) {
		if (std::isnan(point(0)) && std::isnan(point(1)) && std::isnan(point(2))) {
			// All data has been received, we can draw the visualization
			while (viz_window.pollWindowEvent() != -1) {
			}
			viz_window.display();
		} else {
			viz_window.drawRobot(toTransform(point), sf::Color::Red);
		}
	}

	void pose_graph_callback(const pose_t& pose) {
		viz_window.drawRobot(toTransform(pose), sf::Color::Cyan);
	}

	void on_message(ws::connection_hdl handle, message_ptr msg) {
		std::string msg_str = msg->get_payload();
		try {
			json json_msg = json::parse(msg_str);
			if (!json_msg.contains("topic")) {
				return;
			}
			auto topic = json_msg.at("topic");
			rospub::PointPub point_topic;
			rospub::ArrayPub array_topic;
			// check if the topic can be deserialized as a point topic, meaning its payload
			// is a pose_t
			if ((point_topic = topic.get<rospub::PointPub>()) != rospub::PP_INVALID) {
				if (!(json_msg.contains("x") && json_msg.contains("y") &&
					  json_msg.contains("theta"))) {
					std::cerr << "Point topic is missing x, y, or theta field" << std::endl;
					return;
				}
				pose_t pose{get_double_or_nan(json_msg.at("x")),
							get_double_or_nan(json_msg.at("y")),
							get_double_or_nan(json_msg.at("theta"))};
				switch (point_topic) {
					case rospub::PLAN_VIZ:
						plan_callback(pose);
						break;
					case rospub::CURRENT_POSE:
						curr_pose_callback(pose);
						break;
					case rospub::DRIVE_TARGET:
						drive_target_callback(pose);
						break;
					case rospub::PLAN_TARGET:
						plan_target_callback(pose);
						break;
					case rospub::POSE_GRAPH:
						pose_graph_callback(pose);
						break;
					default:
						break;
				}
			}
			// otherwise, check if the topic can be deserialized as an array topic, meaning
			// its payload is a points_t
			else if ((array_topic = topic.get<rospub::ArrayPub>()) != rospub::AP_INVALID) {
				if (!json_msg.contains("points") ||
					json_msg["points"].type() != json::value_t::array) {
					std::cerr << "Array topic is missing points array" << std::endl;
					return;
				}
				points_t points;
				for (const json& curr : json_msg["points"]) {
					if (curr.contains("x") && curr.contains("y") && curr.contains("theta")) {
						double x = get_double_or_nan(curr.at("x"));
						double y = get_double_or_nan(curr.at("y"));
						double theta = get_double_or_nan(curr.at("theta"));
						points.push_back(pose_t{x, y, theta});
					} else {
						std::cerr << "Point missing x, y, or theta field; skipping"
								  << std::endl;
					}
				}
				switch (array_topic) {
					case rospub::LIDAR_SCAN:
						lidar_callback(points);
						break;
					case rospub::LANDMARKS:
						landmarks_callback(points);
						break;
					default:
						break;
				}
			}
		} catch (json::parse_error& e) {
			std::cerr << "Ignoring invalid message \"" << msg_str << "\"" << std::endl;
		}
	}
};

int main(int argc, char** argv) {
	std::string uri = "ws://localhost:9002";
	if (argc == 2) {
		uri = argv[1];
	}
	return PlanViz().run(uri) ? EXIT_SUCCESS : EXIT_FAILURE;
}
