
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "flight_mode.h"


static const char* name = "flight_mode_test";

class FlightModeTest : public rclcpp::Node, public FlightModeBase
{
public:
  FlightModeTest()
  : Node(name), FlightModeBase(*this, name)
  { }

  ~FlightModeTest() = default;

  void checkArmingAndRunConditions(ArmingCheckReporter& reporter) override
  {
	  /* EVENT
	   * @description
	   * Hello from ROS2
	   *
	   * Passed argument: {1:.5}
	   */
	  reporter.armingCheckFailureExt<float>(events::ID("arming_check_from_ros2"), events::Log::Warning,
			  "Arming check from a ROS2 flight mode node through the events interface", 3.141591);

	  reporter.setHealth(health_component_t::avoidance, true, false, false);
  }

  void update() override
  {
	  RCLCPP_INFO(get_logger(), "Update");
	  // TODO: send setpoints
  }

private:
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlightModeTest>());
  rclcpp::shutdown();
  return 0;
}
