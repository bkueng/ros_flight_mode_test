
#include "flight_mode.h"

using namespace std::chrono_literals;

FlightModeBase::FlightModeBase(rclcpp::Node &node,
		const std::string &mode_name)
	: _node(node), _mode_name(mode_name)
{

	if (_mode_name.length() >= sizeof(px4_msgs::msg::RegisterArmingCheckReply::sender_name)) {
		// TODO
	}

    _register_arming_check_reply_sub = _node.create_subscription<px4_msgs::msg::RegisterArmingCheckReply>(
      "/fmu/out/RegisterArmingCheckReply",
      1,
      [this](px4_msgs::msg::RegisterArmingCheckReply::UniquePtr msg) {
      });

    _register_arming_check_request_pub = _node.create_publisher<px4_msgs::msg::RegisterArmingCheckRequest>(
    	"/fmu/in/RegisterArmingCheckRequest", 1);

    _register_nav_mode_reply_sub = _node.create_subscription<px4_msgs::msg::RegisterNavModeReply>(
      "/fmu/out/RegisterNavModeReply",
      1,
      [this](px4_msgs::msg::RegisterNavModeReply::UniquePtr msg) {
      });

    _register_nav_mode_request_pub = _node.create_publisher<px4_msgs::msg::RegisterNavModeRequest>(
    	"/fmu/in/RegisterNavModeRequest", 1);

    _arming_check_reply_pub = _node.create_publisher<px4_msgs::msg::ArmingCheckReply>(
    	"/fmu/in/ArmingCheckReply", 1);

    _arming_check_request_sub = _node.create_subscription<px4_msgs::msg::ArmingCheckRequest>(
      "/fmu/out/ArmingCheckRequest",
      1,
      [this](px4_msgs::msg::ArmingCheckRequest::UniquePtr msg) {
    	RCLCPP_INFO(_node.get_logger(), "Arming check request");
        auto reply = px4_msgs::msg::ArmingCheckReply();
        reply.registration_id = _registration_id;
        reply.request_id = msg->request_id;

        ArmingCheckReporter reporter(reply);
        checkArmingAndRunConditions(reporter);
        _arming_check_reply_pub->publish(reply);
      });

    _vehicle_status_sub = _node.create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/VehicleStatus",
      1,
      [this](px4_msgs::msg::VehicleStatus::UniquePtr msg) {
    	if (msg->nav_state == _nav_mode_id) {
    		if (!_update_timer) {
    			_update_timer = _node.create_wall_timer(0.3s, [this]() { update(); });
    		}
    	} else {
    		if (_update_timer) {
    			_update_timer.reset();
    		}
    	}
      });

	if (!registerMode()) {
		// TODO
	}
}

FlightModeBase::~FlightModeBase()
{
	// TODO: unregister
}

bool FlightModeBase::registerMode()
{
	RCLCPP_INFO(_node.get_logger(), "Registering mode %s", _mode_name.c_str());

	// wait for subscription, it might take a while initially...
	for (int i = 0; i < 100; ++i) {
		if (_register_nav_mode_request_pub->get_subscription_count() > 0 &&
				_register_nav_mode_reply_sub->get_publisher_count() > 0) {
			break;
		}

		usleep(100000);
	}

	// register mode
	rclcpp::WaitSet wait_set;
	wait_set.add_subscription(_register_nav_mode_reply_sub);

	bool ret = false;
	for(int retries=0; retries < 10 && !ret; ++retries) {
		auto message = px4_msgs::msg::RegisterNavModeRequest();
		strcpy((char*)message.mode_name.data(), _mode_name.c_str());
		_register_nav_mode_request_pub->publish(message);

		auto wait_ret = wait_set.wait(200ms);
		if (wait_ret.kind() == rclcpp::WaitResultKind::Ready) {
			px4_msgs::msg::RegisterNavModeReply msg;
			rclcpp::MessageInfo info;
			if (_register_nav_mode_reply_sub->take(msg, info)) {
				if (strcmp((const char*)msg.mode_name.data(), _mode_name.c_str()) == 0) {
					RCLCPP_INFO(_node.get_logger(),"got RegisterNavModeReply, id=%i", msg.nav_mode_id);
					if (msg.nav_mode_id == 0xff) {
						RCLCPP_INFO(_node.get_logger(), "Mode Registration failed");
					} else {
						_nav_mode_id = msg.nav_mode_id;
						ret = true;
					}
				}
			} else {
				RCLCPP_INFO(_node.get_logger(), "no message received");
			}
		} else {
			RCLCPP_INFO(_node.get_logger(), "couldn't wait for message");
		}
	}

	if (ret) {
		ret = registerArmingCheck(_nav_mode_id);
	}
	return ret;
}

bool FlightModeBase::registerArmingCheck(int nav_mode_id)
{
	rclcpp::WaitSet wait_set;
	wait_set.add_subscription(_register_arming_check_reply_sub);

	bool ret = false;
	for(int retries=0; retries < 10 && !ret; ++retries) {
		auto message = px4_msgs::msg::RegisterArmingCheckRequest();
		message.nav_mode_id = nav_mode_id;
		strcpy((char*)message.sender_name.data(), _mode_name.c_str());
		_register_arming_check_request_pub->publish(message);

		auto wait_ret = wait_set.wait(200ms);
		if (wait_ret.kind() == rclcpp::WaitResultKind::Ready) {
			px4_msgs::msg::RegisterArmingCheckReply msg;
			rclcpp::MessageInfo info;
			if (_register_arming_check_reply_sub->take(msg, info)) {
				if (strcmp((const char*)msg.sender_name.data(), _mode_name.c_str()) == 0) {
					RCLCPP_INFO(_node.get_logger(),"got RegisterArmingCheckReply, id=%i", msg.registration_id);
					if (msg.registration_id == 0xff) {
						RCLCPP_INFO(_node.get_logger(), "Registration failed");
					} else {
						_registration_id = msg.registration_id;
						ret = true;
					}
				}
			} else {
				RCLCPP_INFO(_node.get_logger(), "no message received");
			}
		} else {
			RCLCPP_INFO(_node.get_logger(), "couldn't wait for message");
		}
	}

	return ret;
}
