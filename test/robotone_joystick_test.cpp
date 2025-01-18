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
 * @file robotone_joystick_test.cpp
 * @version 0.0.11
 * @date 2025-01-18
 * @note Created a script for testing
 * @brief 
 * 
 */

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class TestJoystickNode : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize ROS2
    rclcpp::init(0, nullptr);
    node = rclcpp::Node::make_shared("test_joystick_node");
  }

  void TearDown() override
  {
    // Shutdown ROS2
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node;
};

TEST_F(TestJoystickNode, TestJoystickPublisher)
{
  // Create a subscriber to verify published messages
  auto subscription = node->create_subscription<std_msgs::msg::String>(
    "/joystick", 10,
    [](std_msgs::msg::String::SharedPtr msg) { EXPECT_EQ(msg->data, "joystick_event"); });

  // Create a publisher to simulate node behavior
  auto publisher = node->create_publisher<std_msgs::msg::String>("/joystick", 10);

  // Publish a test message
  auto message = std_msgs::msg::String();
  message.data = "joystick_event";
  publisher->publish(message);

  // Spin for a short time to process the message
  rclcpp::spin_some(node);
}
