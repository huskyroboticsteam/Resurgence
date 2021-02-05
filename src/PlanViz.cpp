#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "simulator/graphics.h"
#include "simulator/utils.h"

using subscription = rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr;
using std::placeholders::_1;

using namespace std;

class PlanViz : public rclcpp::Node
{
public:
	PlanViz()
		: Node("plan_visualization"),
		  curr_pose({0,0,0}),
		  next_pose({0,0,0}),
		  lidar_scan({}),
		  plan_poses({}),
		  viz_window("Planning visualization")
	{
		plan_sub = this->create_subscription<geometry_msgs::msg::Point>("plan_viz", 10, std::bind(&PlanViz::plan_callback, this, _1));
		curr_pose_sub = this->create_subscription<geometry_msgs::msg::Point>("current_pose", 10, std::bind(&PlanViz::curr_pose_callback, this, _1));
		next_pose_sub = this->create_subscription<geometry_msgs::msg::Point>("next_pose", 10, std::bind(&PlanViz::next_pose_callback, this, _1));
		lidar_sub = this->create_subscription<geometry_msgs::msg::Point>("lidar_scan", 10, std::bind(&PlanViz::lidar_callback, this, _1));
	}

private:
	void curr_pose_callback(const geometry_msgs::msg::Point::SharedPtr message)
	{
		curr_pose = {message->x, message->y, message->z};
		redraw();
	}

	void next_pose_callback(const geometry_msgs::msg::Point::SharedPtr message)
	{
		next_pose = {message->x, message->y, message->z};
		redraw();
	}

	void plan_callback(const geometry_msgs::msg::Point::SharedPtr message)
	{
		if (isnan(message->x) && isnan(message->y) && isnan(message->z))
		{
			plan_poses.clear();
		}
		else if (isinf(message->x) && isinf(message->y) && isinf(message->z))
		{
			redraw();
		}
		else
		{
			plan_poses.push_back({message->x, message->y, message->z});
		}
	}

	void lidar_callback(const geometry_msgs::msg::Point::SharedPtr message)
	{
		if(isnan(message->x) && isnan(message->y) && isnan(message->z))
		{
			lidar_scan.clear();
		}
		else if (isinf(message->x) && isinf(message->y) && isinf(message->z))
		{
			redraw();
		}
		else
		{
			lidar_scan.push_back({message->x, message->y, message->z});
		}
	}

	void redraw()
	{
		viz_window.drawRobot(toTransform(curr_pose), sf::Color::Black);
		viz_window.drawRobot(toTransform(next_pose), sf::Color::Blue);
		for (pose_t plan_pose: plan_poses)
		{
			viz_window.drawRobot(toTransform(plan_pose), sf::Color::Red);
		}
		viz_window.drawPoints(lidar_scan, sf::Color::Red, 3);
		viz_window.display();
	}

	subscription plan_sub;
	subscription curr_pose_sub;
	subscription next_pose_sub;
	subscription lidar_sub;
	pose_t curr_pose;
	pose_t next_pose;
	points_t lidar_scan;
	vector<pose_t> plan_poses;
	MyWindow viz_window;
};

//void stop(int signum)
//{
//	rclcpp::shutdown();
//	raise(SIGTERM);
//}

int main(int argc, char **argv)
{
//	MyWindow viz_window("Planning visualization");

	// Ctrl+C isn't recognized without this line, rclcpp might override the signal
	//signal(SIGINT, stop);

	// Initialize ROS without any commandline arguments
	rclcpp::init(0, nullptr);
//	rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("visualization");
//	subscription plan_sub = node->create_subscription<geometry_msgs::msg::Point>("plan_viz", 10);
//	subscription curr_pose_sub = node->create_subscription<geometry_msgs::msg::Point>("current_pose", 10);
//	subscription next_pose_sub = node->create_subscription<geometry_msgs::msg::Point>("next_pose", 10);
//	subscription lidar_sub = node->create_subscription<geometry_msgs::msg::Point>("lidar_scan", 10);
	rclcpp::spin(std::make_shared<PlanViz>());
	rclcpp::shutdown();
	return 0;
}
