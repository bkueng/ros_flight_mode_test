#pragma once

#include "events.h"

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/register_arming_check_request.hpp"
#include "px4_msgs/msg/register_arming_check_reply.hpp"
#include "px4_msgs/msg/register_nav_mode_request.hpp"
#include "px4_msgs/msg/register_nav_mode_reply.hpp"
#include "px4_msgs/msg/arming_check_request.hpp"
#include "px4_msgs/msg/arming_check_reply.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

#include <string>

class FlightModeBase
{
public:
	FlightModeBase(rclcpp::Node& node, const std::string& mode_name);
	virtual ~FlightModeBase();

	virtual void checkArmingAndRunConditions(ArmingCheckReporter& reporter) = 0;

	virtual void update() = 0;

private:
	bool registerMode();
	bool registerArmingCheck(int nav_mode_id);

	rclcpp::Node& _node;
	const std::string _mode_name;

	rclcpp::TimerBase::SharedPtr _timer;
	rclcpp::Subscription<px4_msgs::msg::RegisterArmingCheckReply>::SharedPtr _register_arming_check_reply_sub;
	rclcpp::Publisher<px4_msgs::msg::RegisterArmingCheckRequest>::SharedPtr _register_arming_check_request_pub;
	uint8_t _registration_id{0};

	rclcpp::Subscription<px4_msgs::msg::RegisterNavModeReply>::SharedPtr _register_nav_mode_reply_sub;
	rclcpp::Publisher<px4_msgs::msg::RegisterNavModeRequest>::SharedPtr _register_nav_mode_request_pub;
	uint8_t _nav_mode_id{0};

	rclcpp::Subscription<px4_msgs::msg::ArmingCheckRequest>::SharedPtr _arming_check_request_sub;
	rclcpp::Publisher<px4_msgs::msg::ArmingCheckReply>::SharedPtr _arming_check_reply_pub;

	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _vehicle_status_sub;
	rclcpp::TimerBase::SharedPtr _update_timer;
};
