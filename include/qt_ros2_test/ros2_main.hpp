#ifndef QT_ROS2_TEST__ROS2_MAIN_HPP_
#define QT_ROS2_TEST__ROS2_MAIN_HPP_

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "roscco_msgs/msg/enable_disable.hpp"
#include "autoware_vehicle_msgs/msg/control_mode_report.hpp"

class Ros2 : public rclcpp::Node
{
public:
  explicit Ros2();
  void pub_roscco_enable();
  void pub_roscco_disable();
  void update_autoware_control(const bool &data);
  void update_twist_controller_trigger(const bool &data);

private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_trigger_;
  rclcpp::Publisher<roscco_msgs::msg::EnableDisable>::SharedPtr pub_enable_disable_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr pub_autoware_control_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_localization_accuracy_;

  rclcpp::TimerBase::SharedPtr timer_;

  roscco_msgs::msg::EnableDisable enable_disable_msg;
  autoware_vehicle_msgs::msg::ControlModeReport autoware_control_msg;
  std_msgs::msg::Bool twist_controller_msg;

  void localization_accuracy_Callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void timer_callback();
};

#endif