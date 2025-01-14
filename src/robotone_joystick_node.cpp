#include <rclcpp/rclcpp.hpp>
#include "robotone_joystick/robotone_joystick.hpp"

int main(int argc, char * argv[])
{
  // Initialization
  rclcpp::init(argc, argv);

  // Set executor
  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // Creat Joystick node
  auto joystick_node =
    std::make_shared<robotone::teleop::RobotoneJoystick>("robotone_joystick_node");

  // Run node
  executor->add_node(joystick_node);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}