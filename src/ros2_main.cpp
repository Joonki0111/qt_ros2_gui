#include "qt_ros2_test/ros2_main.hpp"

ROS2::ROS2() : Node("node")
{
    pub_trigger_ = this->create_publisher<std_msgs::msg::Bool>("/trigger", rclcpp::QoS(1));
    pub_enable_disable_ = this->create_publisher<roscco_msgs::msg::EnableDisable>("/enable_disable", rclcpp::QoS(1));
    pub_autoware_control_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
        "/vehicle/status/control_mode", rclcpp::QoS(1));

    sub_localization_accuracy_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/localization_accuracy", rclcpp::QoS(1), std::bind(
            &ROS2::localization_accuracy_Callback, this, std::placeholders::_1));
    sub_brake_cmd_ = this->create_subscription<roscco_msgs::msg::BrakeCommand>(
        "/brake_command", rclcpp::QoS(1), std::bind(
            &ROS2::brake_cmd_Callback, this, std::placeholders::_1));
    sub_steering_cmd_ = this->create_subscription<roscco_msgs::msg::SteeringCommand>(
        "/steering_command", rclcpp::QoS(1), std::bind(
            &ROS2::steering_cmd_Callback, this, std::placeholders::_1));
    sub_throttle_cmd_ = this->create_subscription<roscco_msgs::msg::ThrottleCommand>(
        "/throttle_command", rclcpp::QoS(1), std::bind(
            &ROS2::throttle_cmd_Callback, this, std::placeholders::_1));      
    sub_gnss_mode_ = this->create_subscription<adma_ros_driver_msgs::msg::AdmaDataScaled>(
        "/genesys/adma/data_scaled", rclcpp::QoS(1), std::bind(
            &ROS2::gnss_mode_Callback, this, std::placeholders::_1));

    auto_mode_client = this->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>("/api/operation_mode/change_to_autonomous");

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ROS2::timer_callback, this));

    brake_position = 0;
    steering_torque = 0;
    throttle_position = 0;
    localization_accuracy_ = 0;
    localization_accuracy_lateral_direction_ = 0;
    autoware_control_msg.mode = 1;
    twist_controller_msg.data = 0;
    gnss_mode = 0;
}

void ROS2::timer_callback()
{
    pub_autoware_control_->publish(autoware_control_msg);
    pub_trigger_->publish(twist_controller_msg);
}

void ROS2::localization_accuracy_Callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    localization_accuracy_ = std::round(msg->data[0] * 1000.0) / 1000.0;
    localization_accuracy_lateral_direction_ = std::round(msg->data[1] * 1000.0) / 1000.0;
}

void ROS2::pub_roscco_enable()
{
    enable_disable_msg.enable_control = 1;
    pub_enable_disable_->publish(enable_disable_msg);
}

void ROS2::pub_roscco_disable()
{
    enable_disable_msg.enable_control = 0;
    pub_enable_disable_->publish(enable_disable_msg);
}

void ROS2::brake_cmd_Callback(const roscco_msgs::msg::BrakeCommand::SharedPtr msg)
{
    brake_position = std::round(msg->brake_position * 100.0) / 100.0;
}

void ROS2::steering_cmd_Callback(const roscco_msgs::msg::SteeringCommand::SharedPtr msg)
{
    steering_torque = std::round(msg->steering_torque * 100.0) / 100.0;
}

void ROS2::throttle_cmd_Callback(const roscco_msgs::msg::ThrottleCommand::SharedPtr msg)
{
    throttle_position = std::round(msg->throttle_position * 100.0) / 100.0;
}

void ROS2::gnss_mode_Callback(const adma_ros_driver_msgs::msg::AdmaDataScaled::SharedPtr msg)
{
    gnss_mode = msg->status.status_gnss_mode;
}

void ROS2::send_autoware_mode_req()
{
    std::shared_ptr<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request> request = 
        std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();
    std::shared_future<std::shared_ptr<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Response>> result = 
        auto_mode_client->async_send_request(request);
}

void ROS2::update_twist_controller_trigger(const bool &data)
{
    twist_controller_msg.data = data;
}

float* ROS2::get_roscco_cmd()
{
    static float roscco_cmd[3];
    roscco_cmd[0] = brake_position;
    roscco_cmd[1] = steering_torque;
    roscco_cmd[2] = throttle_position;
    return roscco_cmd;
}

float* ROS2::get_localization_accuracy()
{
    static float localization_status[2];
    localization_status[0] = localization_accuracy_;
    localization_status[1] = localization_accuracy_lateral_direction_;
    return localization_status;
}

int ROS2::get_gnss_mode()
{
    return gnss_mode;
}
