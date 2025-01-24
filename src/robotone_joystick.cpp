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
 * @version 0.0.14
 * @date 2025-01-24
 * @note Update joystick signal scale and update get paramareters value procedures
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
  RCLCPP_INFO(this->get_logger(), "Initiating RobotOne - RobotOne Joystick Node ...");

  // Declare the parameters with no value
  this->declare_parameter("topic_name", rclcpp::PARAMETER_STRING);
  this->declare_parameter("dev", rclcpp::PARAMETER_STRING);
  this->declare_parameter("debug", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("autorepeat_rate", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("coalesce_interval", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("deadzone", rclcpp::PARAMETER_DOUBLE);

  //Get parameters value
  // Get the value of "debug"
  if (this->get_parameter("debug", config_.debug)) {
    if (config_.debug.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      RCLCPP_INFO_EXPRESSION(
        this->get_logger(), config_.debug.get_value<bool>() == true, "Debug Mode: %s",
        config_.debug.value_to_string().c_str());
    } else {
      RCLCPP_ERROR_EXPRESSION(
        this->get_logger(),
        config_.topic_name.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET,
        "Parameter 'debug' is no set");
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to get parameter 'debug'");
  }

  // Get the value of "topic_name"
  if (this->get_parameter("topic_name", config_.topic_name)) {
    if (config_.topic_name.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      RCLCPP_INFO_EXPRESSION(
        this->get_logger(), config_.debug.get_value<bool>() == true, "Topic name is : %s",
        config_.topic_name.as_string().c_str());
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to get parameter 'topic_name'");
  }

  // Get the value of the "dev"
  if (this->get_parameter("dev", config_.dev)) {
    if (config_.dev.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      RCLCPP_INFO_EXPRESSION(
        this->get_logger(), config_.debug.get_value<bool>() == true,
        "Parameter 'dev' is set to: %s", config_.dev.value_to_string().c_str());
    } else {
      RCLCPP_ERROR_EXPRESSION(
        this->get_logger(), config_.deadzone.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET,
        "Parameter 'dev' is not set");
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to get parameter 'dev'");
  }

  // Get the value of the "coalesce_interval"
  if (this->get_parameter("coalesce_interval", config_.coalesce_interval)) {
    if (config_.coalesce_interval.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      RCLCPP_INFO_EXPRESSION(
        this->get_logger(), config_.debug.get_value<bool>() == true,
        "Parameter 'coalesce_interval' set to: %s",
        config_.coalesce_interval.value_to_string().c_str());
      RCLCPP_WARN_EXPRESSION(
        this->get_logger(), config_.coalesce_interval.get_value<int>() < 0,
        "Parameter coalesce_interval (%s) is less than 0, it is going to be set to 0",
        config_.coalesce_interval.value_to_string().c_str());
      if (config_.coalesce_interval.get_value<int>() < 0) {
        this->set_parameters({rclcpp::Parameter("coalesce_interval", 0)});
      }
    } else {
      RCLCPP_ERROR_EXPRESSION(
        this->get_logger(),
        config_.coalesce_interval.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET,
        "Parameter 'coalesce_interval' is not set");
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to get parameter 'coalesce_interval");
  }

  // Get the value of the "autorepeat_rate"
  if (this->get_parameter("autorepeat_rate", config_.autorepeat_rate)) {
    if (config_.autorepeat_rate.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      RCLCPP_INFO_EXPRESSION(
        this->get_logger(), config_.debug.get_value<bool>() == true,
        "Parameter 'autorepeat_rate' set to: %s",
        config_.autorepeat_rate.value_to_string().c_str());
      RCLCPP_WARN_EXPRESSION(
        this->get_logger(), config_.autorepeat_rate.get_value<int>() < 0,
        "Parameter autorepeat_rate (%s) is less than 0, it is going to be set to 0",
        config_.autorepeat_rate.value_to_string().c_str());
      RCLCPP_WARN_EXPRESSION(
        this->get_logger(), config_.autorepeat_rate.get_value<int>() > 1000,
        "Parameter autorepeat_rate (%s) is greater than 1000, it is going to be set to "
        "1000",
        config_.autorepeat_rate.value_to_string().c_str());
      if (config_.autorepeat_rate.get_value<int>() < 0) {
        this->set_parameters({rclcpp::Parameter("autorepeat_rate", 0)});
      }
    } else {
      RCLCPP_ERROR_EXPRESSION(
        this->get_logger(),
        config_.autorepeat_rate.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET,
        "Parameter 'autorepeat_rate' is not set");
    }

    if (config_.autorepeat_rate.get_value<int>() < 0) {
      this->set_parameters({rclcpp::Parameter("autorepeat_rate", 1000)});
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to get parameter 'autorepeat_rate");
  }

  // Get the value of the "deadzone"
  if (this->get_parameter("deadzone", config_.deadzone)) {
    if (config_.deadzone.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      RCLCPP_INFO_EXPRESSION(
        this->get_logger(), config_.debug.get_value<bool>() == true,
        "Parameter 'deadzone' value is: %s", config_.deadzone.value_to_string().c_str());
      RCLCPP_WARN_EXPRESSION(
        this->get_logger(), config_.deadzone.get_value<double>() < 0,
        "Parameter deadzone (%s) is less than 0, it is going to be set to 0",
        config_.deadzone.value_to_string().c_str());
      if (config_.deadzone.get_value<double>() < 0) {
        this->set_parameters({rclcpp::Parameter("deadzone", 0)});
      } else {
        RCLCPP_ERROR_EXPRESSION(
          this->get_logger(),
          config_.deadzone.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET,
          "Parameter 'deadzone' is not set");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to get parameter 'deadzone'");
    }

    deadzone_scale_ = (config_.deadzone.get_value<double>() * 32767.0);
    scale_ = static_cast<float>(-1.0 / (1.0 - config_.deadzone.get_value<double>()) / 32767.0);

    // Initializes a timer to periodically call the JoystickUpdate() method.
    // NOTE: This line may can be deleted
    int coalesce_interval = config_.coalesce_interval.get_value<int>();

    update_timer_ = create_wall_timer(
      std::chrono::milliseconds(config_.coalesce_interval.get_value<int>()),
      std::bind(&RobotoneJoystick::JoystickUpdate, this));

    // Setup the joystick publisher
    joy_publisher_ =
      this->create_publisher<sensor_msgs::msg::Joy>(config_.topic_name.as_string(), 10);
  }
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
};

/**
 * @brief Opens the joystick device and initializes its properties.
 *
 */
void RobotoneJoystick::OpenJoystick(Joystick & joystick, const std::string & joyPath)
{
  const char path[] = "/dev/input";
  DIR * dev_dir;
  struct dirent * entry;
  struct stat stat_buffer;

  dev_dir = opendir(path);

  // Check if the directory could be opened
  if (dev_dir == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't open %s: %s:", path, std::strerror(errno));
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
    if (ioctl(joystick.file, JSIOCGNAME(sizeof(joystick.name)), joystick.name) < 0) {
      strncpy(joystick.name, "Unkown", sizeof(joystick_.name));
    }

    joystick.is_connected = true;

    RCLCPP_INFO(this->get_logger(), "Found joystick: %s (%s).", joystick.name, joyPath.c_str());
  }

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
    if (ioctl(joystick.file, JSIOCGBUTTONS, &joystick.num_buttons) == -1) {
      perror("JSIOCGBUTTONS error");
      RCLCPP_ERROR(this->get_logger(), "Failed to get the number of buttons.");
      closedir(dev_dir);
      return;
    }

    robotone_joy_msg_.axes.resize(joystick_.num_axes);
    robotone_joy_msg_.buttons.resize(joystick_.num_buttons);

    RCLCPP_INFO(this->get_logger(), "Number of axes: %u", joystick.num_axes);
    RCLCPP_INFO(this->get_logger(), "Number of buttons: %u", joystick.num_buttons);
  }
  // Log no found joystick device
  RCLCPP_ERROR_EXPRESSION(
    this->get_logger(), joystick.is_connected != true, "No connected joystick found: %s",
    joyPath.c_str());

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
void RobotoneJoystick::ReadJoystickInput(Joystick * joystick, Config * config)
{
  ssize_t bytes = read(joystick->file, &joystick->event, sizeof(joystick->event));

  // Reset flags
  joystick->has_axis_event = false;
  joystick->has_buttons_event = false;

  // Check for errors
  if (bytes == -1 && errno != EAGAIN) {
    RCLCPP_ERROR(this->get_logger(), "Error reading joystick input: %s", std::strerror(errno));
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
          RCLCPP_INFO(
            this->get_logger(), "Button %u %s", joystick->event.number,
            joystick->event.value ? "pressed" : "released");
        }
      } else if (joystick->event.type & JS_EVENT_AXIS) {
        // Apply deadzone to axis values
        double axis_value = static_cast<double>(joystick->event.value);
        if (axis_value > deadzone_scale_) {
          axis_value -= deadzone_scale_;
        } else if (axis_value < -deadzone_scale_) {
          axis_value += deadzone_scale_;
        } else {
          axis_value = 0.0;
        }
        joystick->axes[joystick->event.number].value = static_cast<float>(axis_value * scale_);
        joystick->has_axis_event = true;
        // Debug
        if (config->debug.get_value<bool>()) {
          RCLCPP_INFO(
            this->get_logger(), "Axis %u at position %f", joystick->event.number,
            joystick->axes[joystick->event.number].value);
        }
      }
    }
  }
}

/**
 * @brief Continuously monitors joystick events using inotify and processes
 * them.
 *
 * @note Update joystick signal scale and update get paramareters value procedures
 * it should handle any necessary processing or resource management related to
 * the joystick events.
 */
void RobotoneJoystick::JoystickUpdate()
{
  int device_notify;
  int watch_descriptor;
  char buffer[4096];
  struct inotify_event * event;
  joystick_.is_initialized = false;
  rclcpp::Time last_pub = this->now();

  device_notify = inotify_init1(IN_NONBLOCK);

  if (device_notify == -1) {
    RCLCPP_ERROR(this->get_logger(), "Error initializing inotify: %s", std::strerror(errno));
    exit(EXIT_FAILURE);
    return;  // Early exit if inotify fails
  }

  watch_descriptor =
    inotify_add_watch(device_notify, "/dev/input", IN_ATTRIB | IN_DELETE | IN_DELETE_SELF);
  if (watch_descriptor == -1) {
    RCLCPP_ERROR(this->get_logger(), "Error adding inotify watch: %s", std::strerror(errno));
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

      RCLCPP_INFO(
        this->get_logger(), "Joystick not connected, trying to connect every second: %s",
        config_.dev.as_string().c_str());
      sleep(1.0);

      OpenJoystick(joystick_, config_.dev.as_string().c_str());
    }

    if (joystick_.is_connected) {
      event = reinterpret_cast<struct inotify_event *>(buffer);

      RCLCPP_ERROR_EXPRESSION(
        this->get_logger(), (event->mask & IN_DELETE || event->mask & IN_DELETE_SELF) == 1,
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
        if (
          RCL_NS_TO_MS(duration_.nanoseconds()) >=
          (1000 / config_.autorepeat_rate.get_value<int>())) {
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
