/**
 * Copyright (C) 2025 A. Freire
 *
 * This file is part of RobotOne
 *
 * RobotOne is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RobotOne is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with [Project Name].  If not, see <http://www.gnu.org/licenses/>.
 *
 * @file robotone_joystick_node.cpp
 * @version 0.0.12
 * @date 2025-01-19
 * @note Deleted test folder
 * @brief
 *
 */
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
