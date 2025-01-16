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
 * along with RobotOne.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @file robotone_joystick.cpp
 * @version 0.0.0-alpha.1
 * @date 2025-01-16
 * @brief This C++ file implements a ROS2 node RobotoneJoystick to handle
 * joystick inputs for the Robotone project. It reads joystick
 * events->button_state and axes), manages device connections, and publishes the
 * data using ROS2 messages. Additionally, it handles feedback from the system
 * to c control joystick haptic responses.
 *
 */
#include <dirent.h>
#include <fcntl.h>
#include <sys/inotify.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstring>
#include <rclcpp/utilities.hpp>
#include "robotone_joystick/robotone_joystick.hpp"

using namespace robotone::teleop;

/**
 * @brief Construct a new Robotone Joystick and define the parameters
 */
RobotoneJoystick::RobotoneJoystick(const std::string & name)
: rclcpp::Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
  RCLCPP_INFO(this->get_logger(),
              "Initiating RobotOne - RobotOne Joystick Node ...");

  // Declare the parameters with no value
  this->declare_parameter("topic_name", rclcpp::PARAMETER_STRING);
  this->declare_parameter("dev", rclcpp::PARAMETER_STRING);
  this->declare_parameter("debug", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("autorepeat_rate", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("coalesce_interval", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("deadzone", rclcpp::PARAMETER_DOUBLE);

  // Get parameters value
  this->get_parameter("debug", config_.debug);
  this->get_parameter("autorepeat_rate", config_.autorepeat_rate);
  this->get_parameter("coalesce_interval", config_.coalesce_interval);
  this->get_parameter("deadzone", config_.deadzone);

  // Get the value of "topic_name"
  if (this->get_parameter("topic_name", config_.topic_name)) {
    if (config_.topic_name.get_type() ==
      rclcpp::ParameterType::PARAMETER_STRING)
    {
      RCLCPP_INFO_EXPRESSION(
          this->get_logger(), config_.debug.get_value<bool>() == true,
          "Topic name is : %s", config_.topic_name.as_string().c_str());
    } else if (config_.topic_name.get_type() ==
      rclcpp::ParameterType::PARAMETER_NOT_SET)
    {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'topic_name' is not set");
    } else if (config_.topic_name.get_value<std::string>().empty()) {
      RCLCPP_ERROR(this->get_logger(), "Parameter topic name is empty");
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), " Failed to get parameter 'topic_name'");
  }

  // Get the value of the "dev" parameter
  if (this->get_parameter("dev", config_.dev)) {
    if (config_.dev.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'dev' is not set.");
    } else {
      std::string dev_value = config_.dev.get_value<std::string>();
      if (dev_value.empty()) {
        RCLCPP_INFO(this->get_logger(), "Parameter 'dev' is empty.");
      } else {
        RCLCPP_INFO(this->get_logger(), "Parameter 'dev' has a value: %s",
                    dev_value.c_str());
      }
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to get parameter 'dev'");
  }

  // Get the value of the "coalesce_interval" parameter
  // TODO we may need to use it to reduce the number of messages
  int coalesce_interval = config_.coalesce_interval.get_value<int>();

  RCLCPP_WARN_EXPRESSION(
      this->get_logger(), config_.coalesce_interval.get_value<int>() < 0,
      "%s: coalesce_interval (%s) is less than 0, it is going to be set to 0",
      name.c_str(), config_.coalesce_interval.value_to_string().c_str());

  if (config_.coalesce_interval.get_value<int>() < 0) {
    double zero_ = 0;
    std::vector<rclcpp::Parameter> parameters;
    parameters.push_back(rclcpp::Parameter("coalesce_interval", zero_));
    this->set_parameters(parameters);
  }

  // we may need to use auto repeat
  RCLCPP_WARN_EXPRESSION(
      this->get_logger(), config_.autorepeat_rate.get_value<int>() < 0,
      "%s: autorepeat_rate (%s) is less than 0, it is going to be set to 0",
      name.c_str(), config_.autorepeat_rate.value_to_string().c_str());

  RCLCPP_WARN_EXPRESSION(
      this->get_logger(), config_.autorepeat_rate.get_value<int>() > 1000,
      "%s: autorepeat_rate (%s) is greater than 1000, it is going to be set to "
      "1000",
      name.c_str(), config_.autorepeat_rate.value_to_string().c_str());

  if (config_.autorepeat_rate.get_value<int>() < 0) {
    int zero_ = 0;
    std::vector<rclcpp::Parameter> parameters;
    parameters.push_back(rclcpp::Parameter("autorepeat_rate", zero_));
    this->set_parameters(parameters);
  }

  if (config_.autorepeat_rate.get_value<int>() < 0) {
    int thousand_ = 1000;
    std::vector<rclcpp::Parameter> parameters;
    parameters.push_back(rclcpp::Parameter("autorepeat_rate", thousand_));
    this->set_parameters(parameters);
  }

  RCLCPP_INFO(this->get_logger(), "Parameter autorepeat_rate has a value: %s",
              config_.autorepeat_rate.value_to_string().c_str());

  RCLCPP_INFO_EXPRESSION(
      this->get_logger(), config_.debug.get_value<bool>() == true,
      "Debug Mode: %s", config_.debug.value_to_string().c_str());

  /**
   * @brief Initializes a timer to periodically call the JoystickUpdate()
   * method. This line creates a wall timer that triggers the JoystickUpdate()
   * method of the RobotoneJoystick class at regular intervals. The timer is set
   * to fire every 10 milliseconds, allowing the JoystickUpdate() method to
   * continuously monitor joystick events and update the corresponding data
   * structures. The std::bind function is used to bind the method
   * JoystickUpdate() to the timer callback, ensuring its execution within the
   * context of the RobotoneJoystick instance.
   */
  update_timer_ =
    create_wall_timer(std::chrono::milliseconds(coalesce_interval),
                        std::bind(&RobotoneJoystick::JoystickUpdate, this));

  // Setup the joystick publisher
  joy_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>(
      config_.topic_name.as_string(), 10);
}
/**
 * @brief Destroy the Robotone Joystick:: Robotone Joystick object
 *
 */
RobotoneJoystick::~RobotoneJoystick()
{
  // Close the joystick
  if (joystick_.is_connected) {
    CloseJoystick(joystick_);
  }
}

/**
 * @brief Opens the joystick device and initializes its properties.
 *
 */
void RobotoneJoystick::OpenJoystick(
  Joystick & joystick,
  const std::string & joyPath)
{
  const char path[] = "/dev/input";
  DIR *dev_dir;
  struct dirent *entry;
  struct stat stat_buffer;

  dev_dir = opendir(path);

  // Check if the directory could be opened
  if (dev_dir == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't open %s: %s:", path,
                 std::strerror(errno));
    closedir(dev_dir);
    return;
  }

  // Iterate through each entry in the directory
  while ((entry = readdir(dev_dir)) != nullptr) {
    if (strncmp(entry->d_name, "js", 2) != 0) {
      continue;
    }
    if (stat(joyPath.c_str(), &stat_buffer) == -1) {
      continue;
    }
    joystick.file = open(joyPath.c_str(), O_RDONLY | O_NONBLOCK);
    if (joystick.file == -1) {
      continue;
    }
    // Retrieve the name of the joystick
    if (ioctl(joystick.file, JSIOCGNAME(sizeof(joystick.name)), joystick.name) <
      0)
    {
      strncpy(joystick.name, "Unkown", sizeof(joystick_.name));
    }

    joystick.is_connected = true;

    RCLCPP_INFO(this->get_logger(), "Found joystick: %s (%s).", joystick.name,
                joyPath.c_str());
  }
  // TODO verify if we need those data
  // Set up axis properties if the joystick is connected
  if (joystick.is_connected) {
    // Retrieve axis information for the connected joystick
    // Retrieve the number of supported Axes
    if (ioctl(joystick.file, JSIOCGAXES, &joystick.num_axes) == -1) {
      perror("JSIOCGAXES error");
      RCLCPP_ERROR(this->get_logger(), "Failed to get the number of axes.");
      closedir(dev_dir);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Number of axes: %u", joystick.num_axes);
  }
  // Log no found joystick device
  RCLCPP_ERROR_EXPRESSION(this->get_logger(), joystick.is_connected != true,
                          "No connected joystick found: %s", joyPath.c_str());

  closedir(dev_dir);
}

/**
 * @brief Close the file descriptor associated with the joystick device
 */
void RobotoneJoystick::CloseJoystick(Joystick & joystick)
{
  // Close the file descriptor associated with the joystick device
  close(joystick.file);
  // Reset the connection status flag
  joystick.is_connected = false;
}

/**
 * @brief Reads input from the joystick file descriptor and processes joystick
 * events.
 *
 */
void RobotoneJoystick::ReadJoystickInput(Joystick *joystick, Config *config)
{
  ssize_t bytes =
    read(joystick->file, &joystick->event, sizeof(joystick->event));

  // Reset flags
  joystick->has_axis_event = false;
  joystick->has_buttons_event = false;

  // Min and Max
  double min_value = -32767.0;
  double max_value = 32767.0;

  // Calculate deadzone threshold
  double deadzone = config->deadzone.get_value<double>();
  double deadzone_threshold = deadzone * max_value;

  // Check for errors
  if (bytes == -1 && errno != EAGAIN) {
    RCLCPP_ERROR(this->get_logger(), "Error reading joystick input: %s",
                 std::strerror(errno));
    return;
  }

  if (bytes == sizeof(joystick->event)) {
    if (joystick->event.type & JS_EVENT_INIT) {
      // Handle joystick initialization event
      joystick_.is_initialized = true;
    } else if (joystick->is_initialized) {
      // Process joystick events after initialization
      if (joystick->event.type & JS_EVENT_BUTTON) {
        joystick->button_state[joystick->event.number] = joystick->event.value;
        joystick->has_buttons_event = true;
        // Debug
        if (config->debug.get_value<bool>()) {
          RCLCPP_INFO(this->get_logger(), "Button %u %s",
                      joystick->event.number,
                      joystick->event.value ? "pressed" : "released");
        }
      } else if (joystick->event.type & JS_EVENT_AXIS) {
        // Apply deadzone to axis values
        short axis_value = joystick->event.value;
        if (std::abs(joystick->event.value) < deadzone_threshold) {
          axis_value = 0.0;
        } else {
          // Normalize the axis value
          double normalized_value =
            2 * (axis_value - min_value) / (max_value - min_value) - 1;
          double scaled_value = normalized_value;
          axis_value = scaled_value;
        }
        joystick->axes[joystick->event.number].value = axis_value;
        joystick->has_axis_event = true;
        // Debug
        if (config->debug.get_value<bool>()) {
          RCLCPP_INFO(this->get_logger(), "Axis %u at position %d",
                      joystick->event.number, joystick->event.value);
        }
      }
    }
  }
}

/**
 * @brief Continuously monitors joystick events using inotify and processes
 * them.
 *
 * @note The openJoystick function is called for each joystick event read, and
 * it should handle any necessary processing or resource management related to
 * the joystick events.
 */
void RobotoneJoystick::JoystickUpdate()
{
  int device_notify;
  int watch_descriptor;
  char buffer[4096];
  struct inotify_event *event;
  joystick_.is_initialized = false;
  rclcpp::Time last_pub = this->now();

  device_notify = inotify_init1(IN_NONBLOCK);

  if (device_notify == -1) {
    RCLCPP_ERROR(this->get_logger(), "Error initializing inotify: %s",
                 std::strerror(errno));
    exit(EXIT_FAILURE);
    return;  // Early exit if inotify fails
  }

  watch_descriptor = inotify_add_watch(device_notify, "/dev/input",
                                       IN_ATTRIB | IN_DELETE | IN_DELETE_SELF);
  if (watch_descriptor == -1) {
    RCLCPP_ERROR(this->get_logger(), "Error adding inotify watch: %s",
                 std::strerror(errno));
    close(device_notify);
    exit(EXIT_FAILURE);
    return;  // Early exit if watch fails
  }

  // Look if there is any new connection
  OpenJoystick(joystick_, config_.dev.as_string().c_str());

  while (rclcpp::ok()) {
    // We keep trying to connect every second until node shutdown or have a
    // joystick connected.
    while (!joystick_.is_connected) {
      if (!rclcpp::ok()) {
        goto shutdown;
      }

      RCLCPP_INFO(this->get_logger(),
                  "Joystick not connected, trying to connect every second: %s",
                  config_.dev.as_string().c_str());
      sleep(1.0);

      OpenJoystick(joystick_, config_.dev.as_string().c_str());
    }

    if (joystick_.is_connected) {
      event = reinterpret_cast<struct inotify_event *>(buffer);

      RCLCPP_ERROR_EXPRESSION(
          this->get_logger(),
        (event->mask & IN_DELETE || event->mask & IN_DELETE_SELF) == 1,
          "Joystick connection error");

      if (event->mask & IN_DELETE || event->mask & IN_DELETE_SELF) {
        joystick_.is_connected = false;
        close(device_notify);
      } else {
        ReadJoystickInput(&joystick_, &config_);

        if (joystick_.has_buttons_event || joystick_.has_axis_event) {
          if (joystick_.has_buttons_event) {
            if (joystick_.event.number >= robotone_joy_msg_.buttons.size()) {
              // Resize the button vector and initialize new elements to 0.0
              robotone_joy_msg_.buttons.resize(joystick_.event.number + 1, 0.0);
            }
            robotone_joy_msg_.buttons[joystick_.event.number] =
              (joystick_.button_state[joystick_.event.number] ? 1 : 0);
          }
          if (joystick_.has_axis_event) {
            if (joystick_.event.number >= robotone_joy_msg_.axes.size()) {
              // Resize the axis vector and initialize new elements to 0.0
              robotone_joy_msg_.axes.resize(joystick_.event.number + 1, 0.0);
            }
            robotone_joy_msg_.axes[joystick_.event.number] =
              (joystick_.axes[joystick_.event.number].value);
          }
          robotone_joy_msg_.header.stamp = this->now();
          robotone_joy_msg_.header.frame_id = config_.dev.as_string().c_str();
          joy_publisher_->publish(robotone_joy_msg_);

          last_pub = this->now();
        }
      }

      if (config_.autorepeat_rate.get_value<int>() > 0) {
        rclcpp::Time now = this->now();
        rclcpp::Duration duration_ = now - last_pub;
        if (RCL_NS_TO_MS(duration_.nanoseconds()) >=
          (1000 / config_.autorepeat_rate.get_value<int>()))
        {
          robotone_joy_msg_.header.stamp = this->now();
          robotone_joy_msg_.header.frame_id = config_.dev.as_string().c_str();
          joy_publisher_->publish(robotone_joy_msg_);
          last_pub = this->now();
        }
      }

      if (!rclcpp::ok()) {
        goto shutdown;
      }
    }
  }

shutdown:
  RCLCPP_INFO(this->get_logger(), "Node %s shutdown", this->get_name());
  close(device_notify);
}
