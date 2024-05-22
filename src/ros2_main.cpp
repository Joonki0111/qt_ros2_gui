#include "qt_ros2_test/ros2_main.hpp"

// RCLCPP_INFO(rclcpp::get_logger("test"),"%d", main_Btn_state);

Ros2::Ros2() : Node("node")
{
    pub_trigger_ = this->create_publisher<std_msgs::msg::Bool>("/trigger", rclcpp::QoS(1));
    pub_enable_disable_ = this->create_publisher<roscco_msgs::msg::EnableDisable>("/enable_disable", rclcpp::QoS(1));
    pub_autoware_control_ = this->create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>(
        "/vehicle/status/control_mode", rclcpp::QoS(1));

    sub_localization_accuracy_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/localization_accuracy", rclcpp::QoS(1), std::bind(
            &Ros2::localization_accuracy_Callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&Ros2::timer_callback, this));
}

void Ros2::timer_callback()
{
    pub_autoware_control_->publish(autoware_control_msg);
    pub_trigger_->publish(twist_controller_msg);
}

void Ros2::localization_accuracy_Callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    // localization_accuracy_ = msg->data[0];
    // localization_accuracy_lateral_direction_ = msg->data[1];
}

void Ros2::pub_roscco_enable()
{
    enable_disable_msg.enable_control = 1;
    pub_enable_disable_->publish(enable_disable_msg);
}

void Ros2::pub_roscco_disable()
{
    enable_disable_msg.enable_control = 0;
    pub_enable_disable_->publish(enable_disable_msg);
}

void Ros2::update_autoware_control(const bool &data)
{
    autoware_control_msg.mode = data;
}

void Ros2::update_twist_controller_trigger(const bool &data)
{
    twist_controller_msg.data = data;
}
