#ifndef QT_ROS2_TEST__ROS2_MAIN_HPP_
#define QT_ROS2_TEST__ROS2_MAIN_HPP_

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "roscco_msgs/msg/enable_disable.hpp"
#include "roscco_msgs/msg/brake_command.hpp"
#include "roscco_msgs/msg/steering_command.hpp"
#include "roscco_msgs/msg/throttle_command.hpp"
#include "autoware_vehicle_msgs/msg/control_mode_report.hpp"

class ROS2 : public rclcpp::Node
{
public:
  explicit ROS2();
  void pub_roscco_enable();
  void pub_roscco_disable();
  void update_autoware_control(const bool &data);
  void update_twist_controller_trigger(const bool &data);
  float* get_roscco_cmd();

private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_trigger_;
  rclcpp::Publisher<roscco_msgs::msg::EnableDisable>::SharedPtr pub_enable_disable_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr pub_autoware_control_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_localization_accuracy_;
  rclcpp::Subscription<roscco_msgs::msg::BrakeCommand>::SharedPtr sub_brake_cmd_;
  rclcpp::Subscription<roscco_msgs::msg::SteeringCommand>::SharedPtr sub_steering_cmd_;
  rclcpp::Subscription<roscco_msgs::msg::ThrottleCommand>::SharedPtr sub_throttle_cmd_;

  rclcpp::TimerBase::SharedPtr timer_;

  roscco_msgs::msg::EnableDisable enable_disable_msg;
  autoware_vehicle_msgs::msg::ControlModeReport autoware_control_msg;
  std_msgs::msg::Bool twist_controller_msg;

  float localization_accuracy_;
  float localization_accuracy_lateral_direction_;
  float brake_position;
  float steering_torque;
  float throttle_position;

  void localization_accuracy_Callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void brake_cmd_Callback(const roscco_msgs::msg::BrakeCommand::SharedPtr msg);
  void steering_cmd_Callback(const roscco_msgs::msg::SteeringCommand::SharedPtr msg);
  void throttle_cmd_Callback(const roscco_msgs::msg::ThrottleCommand::SharedPtr msg);
  void timer_callback();
};

#endif