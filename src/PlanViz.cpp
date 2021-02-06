#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "simulator/graphics.h"
#include "simulator/utils.h"

using std::placeholders::_1;

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
		plan_sub = this->create_subscription<geometry_msgs::msg::Point>("plan_viz", 100, std::bind(&PlanViz::plan_callback, this, _1));
		curr_pose_sub = this->create_subscription<geometry_msgs::msg::Point>("current_pose", 100, std::bind(&PlanViz::curr_pose_callback, this, _1));
		next_pose_sub = this->create_subscription<geometry_msgs::msg::Point>("next_pose", 100, std::bind(&PlanViz::next_pose_callback, this, _1));
		lidar_sub = this->create_subscription<geometry_msgs::msg::Point>("lidar_scan", 100, std::bind(&PlanViz::lidar_callback, this, _1));
		viz_window.display();
	}

private:
	void curr_pose_callback(const geometry_msgs::msg::Point::SharedPtr message)
	{
		curr_pose = {message->x, message->y, message->z};
	}

	void next_pose_callback(const geometry_msgs::msg::Point::SharedPtr message)
	{
		next_pose = {message->x, message->y, message->z};
	}

	void lidar_callback(const geometry_msgs::msg::Point::SharedPtr message)
	{
		lidar_scan.push_back({message->x, message->y, message->z});
	}

	void plan_callback(const geometry_msgs::msg::Point::SharedPtr message)
	{
		// This function also handles start and end messages
		if (std::isnan(message->x) && std::isnan(message->y) && std::isnan(message->z))
		{
			// Clear data from last visualization
			lidar_scan.clear();
			plan_poses.clear();
			// Reset target pose so it's not shown if it isn't sent
			next_pose = {NAN, NAN, NAN};
		}
		else if (std::isinf(message->x) && std::isinf(message->y) && std::isinf(message->z))
		{
			// All data has been received, we can draw the visualization
			draw();
		}
		else
		{
			plan_poses.push_back({message->x, message->y, message->z});
		}
	}

	void draw()
	{
		while (viz_window.pollWindowEvent() != -1) {}
		viz_window.drawRobot(toTransform(curr_pose), sf::Color::Black);
		// Don't draw target pose if it wasn't sent
		if (!std::isnan(next_pose(0)) && !std::isnan(next_pose(1)) && !std::isnan(next_pose(2)))
		{
			viz_window.drawRobot(toTransform(next_pose), sf::Color::Blue);
		}
		for (pose_t plan_pose: plan_poses)
		{
			viz_window.drawRobot(toTransform(plan_pose), sf::Color::Red);
		}
		viz_window.drawPoints(lidar_scan, sf::Color::Red, 3);
		viz_window.display();
	}

	rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr plan_sub;
	rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr curr_pose_sub;
	rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr next_pose_sub;
	rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr lidar_sub;
	pose_t curr_pose;
	pose_t next_pose;
	points_t lidar_scan;
	std::vector<pose_t> plan_poses;
	MyWindow viz_window;
};

int main(int argc, char **argv)
{
	// Initialize ROS without any commandline arguments
	rclcpp::init(0, nullptr);

	// Start reading data
	rclcpp::spin(std::make_shared<PlanViz>());

	// Shutdown ROS when done reading data
	rclcpp::shutdown();
	return 0;
}
