/**
 * Copyright (C) 2025 A. Freire
 *
 * This file is part of RobotOne
 *
 * RobotOne is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as should_published by
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
 * @file robotone_joystick.hpp
 * @version 0.0.1
 * @date 2025-01-16
 * @note No notes add by developer
 * @brief This C++ header file defines a ROS2 node called RobotoneJoystick to
 * handle joystick inputs for the Robotone project. The node manages joystick
 * connections, reads input data (buttons and axes), and publishes the data as
 * ROS2 messages. It also includes methods for feedback handling, input reading,
 * and device management.
 *
 */
#ifndef ROBOTONE_JOYSTICK__ROBOTONE_JOYSTICK_HPP_
#define ROBOTONE_JOYSTICK__ROBOTONE_JOYSTICK_HPP_

#include <linux/joystick.h>

#include <memory>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback_array.hpp>
#include <string>

/**
 * @defgroup joystick_methods Joystick Methods
 * @brief Group for joystick-related methods
 *
 */
namespace robotone
{
namespace teleop
{
/**
 * @brief Class to handle joystick input and publish events for the Robotone
 * project.
 *
 */
class RobotoneJoystick : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Robotone Joystick object
   *
   * @param name Name of ROS2 Node
   */
  RobotoneJoystick(const std::string & name);

  /**
   * @brief Destroy the Robotone Joystick object
   *
   * @param none
   *
   */
  ~RobotoneJoystick();

public:
  /**
   * @brief Struct to store axis configuration, including minimum, maximum, and
   * current values.
   *
   */
  struct AxisConfig
  {
    short min;    ///< Minimum value of the axis.
    short max;    ///< Maximum value of the axis.
    short value;  ///< Current value of the axis.
  };

  /**
   * @brief Struct to represent the state of the joystick, including buttons and
   * axes.
   *
   */
  struct Joystick
  {
    static const int kMaxAxes = 32;     ///< Maximum number of axis supported.
    static const int kMaxButtons = 32;  ///< Maximum number of buttons supported.

    bool has_axis_event;             ///< Flag to indicate if an axis event ocurred.
    bool has_buttons_event;          ///< Flag to indicate if a button event ocurred.
    bool should_publish;             ///< Flag to indicate if data should be published.
    bool is_connected;               ///< Flag to indicate if joystick is connected.
    bool is_initialized;             ///< Flag to indicate if joystick is is_initialized.
    bool button_state[kMaxButtons];  ///< Array to store button states.
    AxisConfig axes[kMaxAxes];       ///< Array to store axis configurations.
    char name[128];                  ///< Name of the joystick device.
    int file;                        ///< File descriptor for the joystick device.
    int num_axes;                    ///< Number of axes detected.
    struct js_event event;           ///< Struct to store the latest joystick event.

    /**
     * @brief Construct a new Joystick object
     *
     */
    Joystick()
    : has_axis_event(false),
      has_buttons_event(false),
      should_publish(false),
      is_connected(false),
      is_initialized(false),
      file(-1)
    {
      std::memset(button_state, 0, sizeof(button_state));
      std::memset(name, 0, sizeof(name));

      for (int i = 0; i < kMaxAxes; ++i) {
        axes[i] = {0, 0, 0};
      }

      std::memset(&event, 0, sizeof(event));
    }
  };
  /**
   * @brief Struct to store configuration parameters for the joystick.
   *
   */
  struct Config
  {
    rclcpp::Parameter topic_name;         ///< Name of the topic to publish joystick data.
    rclcpp::Parameter debug;              ///< Debug flag.
    rclcpp::Parameter dev;                ///< Joystick device path
    rclcpp::Parameter autorepeat_rate;    ///< Rate at which to autorepeat joystick events.
    rclcpp::Parameter coalesce_interval;  ///< Interval to coalesce multiple events.
    rclcpp::Parameter deadzone;           ///< Deadzone value for the joystick.
  };

private:
  // TODO: Needs to be implemented
  /**
   *
   * @brief Set the Feedback object
   *
   * @param msg
   * @ingroup joystick_methods
   */
  void SetFeedback(const std::shared_ptr<sensor_msgs::msg::JoyFeedbackArray> msg);

  /**
   * @brief Function to update joystick inputs
   *
   * This function initializes an inotify instance to monitor changes in the
   * "/dev/input" directory, specifically for joystick-related events such as
   * device addition or removal. It creates a separate thread to read inotify
   * events in a loop and processes them using the openJoystick function. The
   * function runs continuously until the node is shut down or an error occurs
   * during event reading.
   *
   * In case of an initialization failure or an error during event reading,
   * appropriate error messages are logged, and cleanup is performed by closing
   * the inotify descriptor.
   *
   * @param none
   * @ingroup joystick_methods
   */
  void JoystickUpdate();

  /**
   * @brief Reads input from the joystick file descriptor and processes joystick
   * events.
   *
   * This function reads data from the joystick file descriptor and interprets
   * it as joystick events. It sets flags to indicate axis and button events and
   * updates corresponding data structures accordingly.
   *
   * @param joystick A pointer to the Joystick struct containing joystick
   * information.
   * @param config A pointer to the Config struct containing configuration
   * parameters.
   * @ingroup joystick_methods
   */
  void ReadJoystickInput(Joystick * joystick, Config * config);

  /**
   * @brief Opens the joystick device and initializes its properties.
   *
   * This function attempts to open the joystick device specified by 'joyPath'
   * and initializes its properties. It iterates through the directory
   * '/dev/input' to find joystick devices starting with 'js'. For each found
   * device, it checks if it matches the specified 'joyPath' and opens the
   * device in non-blocking mode. If successful, it retrieves the joystick name
   * and marks the joystick as connected.
   *
   * Additionally, it retrieves axis information for the connected joystick and
   * sets up the axis properties including minimum and maximum values.
   *
   * @param joystick A reference to the Joystick struct to store joystick
   * information.
   * @param joyPath The path of the joystick device to be opened.
   * @ingroup joystick_methods
   */
  void OpenJoystick(Joystick & joystick, const std::string & joyPath);

  /**
   * @brief Close the joystick device.
   *
   * @param joystick Reference to the Joystick object to be closed.
   * @ingroup joystick_methods
   */
  void CloseJoystick(Joystick & joystick);

private:
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr
    joy_publisher_;  ///< Publisher for joystick messages.
  rclcpp::Subscription<sensor_msgs::msg::JoyFeedbackArray>::SharedPtr
    joy_subscriber_;                           ///< Subscriber for joystick feedback.
  sensor_msgs::msg::Joy robotone_joy_msg_;     ///< Message to store joystick data.
  rclcpp::TimerBase::SharedPtr update_timer_;  ///< Timer for periodic updates.
  Joystick joystick_;                          ///< Joystick object to manage joystick state.
  Config config_;                              ///< Configuration object to store parameters.
};
}  // end namespace teleop

}  // end namespace robotone

#endif  // ROBOTONE_JOYSTICK__ROBOTONE_JOYSTICK_HPP_
