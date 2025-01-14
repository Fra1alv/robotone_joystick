#ifndef ROBOTONE_JOYSTICK__ROBOTONE_JOYSTICK_HPP_
#define ROBOTONE_JOYSTICK__ROBOTONE_JOYSTICK_HPP_

#include <linux/joystick.h>
#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback_array.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

namespace robotone
{
namespace teleop
{
class RobotoneJoystick : public rclcpp::Node
{
public:
  RobotoneJoystick(const std::string& name);
  ~RobotoneJoystick();

public:
  struct AxisConfig
  {
    short min;
    short max;
    short value;
  };

  struct Joystick
  {
    static const int kMaxAxes = 32;
    static const int kMaxButtons = 32;

    bool axis_event;
    bool buttons_event;
    bool publish;
    bool connected;
    bool initialized;
    bool buttons[kMaxButtons];
    AxisConfig axes[kMaxAxes];
    char name[128];
    int file;
    int num_axes;
    struct js_event event;

    Joystick() : axis_event(false), buttons_event(false), publish(false), connected(false), initialized(false), file(-1)
    {
      std::memset(buttons, 0, sizeof(buttons));
      std::memset(name, 0, sizeof(name));

      for (int i = 0; i < kMaxAxes; ++i)
      {
        axes[i] = { 0, 0, 0 };
      }

      std::memset(&event, 0, sizeof(event));
    }
  };
  struct Config
  {
    rclcpp::Parameter topic_name;
    rclcpp::Parameter debug;
    rclcpp::Parameter dev;
    rclcpp::Parameter autorepeat_rate;
    rclcpp::Parameter coalesce_interval;
    rclcpp::Parameter deadzone;
  };

private:
  void SetFeedback(const std::shared_ptr<sensor_msgs::msg::JoyFeedbackArray> msg);

  /**
   * @brief Function to update joystick inputs
   */
  void JoystickUpdate();

  /**
   * @brief Function to read joystick inputs
   *
   * @param joystick
   */
  void ReadJoystickInput(Joystick* joystick, Config* config);

  /**
   * @brief Creat a joystick object
   *
   * @param fileName
   *
   */
  void OpenJoystick(Joystick& joystick, const std::string& joyPath);

  /**
   * @brief Close Joystick
   *
   * @param joystick
   */
  void CloseJoystick(Joystick& joystick);

private:
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JoyFeedbackArray>::SharedPtr joy_subscriber_;
  sensor_msgs::msg::Joy robotone_joy_msg_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  Joystick joystick_;
  Config config_;
};
}  // end namespace teleop

}  // end namespace robotone

#endif  // ROBOTONE_JOYSTICK__ROBOTONE_JOYSTICK_HPP_