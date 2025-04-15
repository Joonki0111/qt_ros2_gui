#include "qt_ros2_gui/ros2_main.hpp"

ROS2::ROS2() : Node("qt_ros2_node")
{
    ROSCCO_enable_disable_pub_ = this->create_publisher<roscco_msgs::msg::EnableDisable>(
        "/roscco/enable_disable", rclcpp::QoS(1));

    localization_accuracy_sub_ = this->create_subscription<autoware_localization_msgs::msg::LocalizationAccuracy>(
        "/localization_accuracy", rclcpp::QoS(1), std::bind(
            &ROS2::LocalizationAccuracyCallback, this, std::placeholders::_1));   
    adma_gnss_mode_sub_ = this->create_subscription<std_msgs::msg::Int8>
        ("/genesys/adma/gnss_mode", 10, std::bind(&ROS2::ADMADataCallback, this, std::placeholders::_1));
    ROSCCO_status_sub_ = this->create_subscription<roscco_msgs::msg::RosccoStatus>
        ("/roscco/status", rclcpp::QoS(1), std::bind(&ROS2::ROSCCOStatusCallback, this, std::placeholders::_1));
    component_status_sub_ = this->create_subscription<autoware_system_msgs::msg::ComponentStatus>
        ("/system/status/component_status", rclcpp::QoS(1), std::bind(&ROS2::ComponentStatusCallback, this, std::placeholders::_1));

    AW_auto_client = this->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>("/api/operation_mode/change_to_autonomous");
    AW_stop_client = this->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>("/api/operation_mode/change_to_stop");

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ROS2::TimerCallback, this));
}

void ROS2::TimerCallback(){}

void ROS2::LocalizationAccuracyCallback(const autoware_localization_msgs::msg::LocalizationAccuracy::SharedPtr msg)
{
    localization_accuracy_long_radius_ = std::round(msg->long_radius * 1000.0) / 1000.0;
    localization_accuracy_lateral_direction_ = std::round(msg->lateral_accuracy * 1000.0) / 1000.0;
}

void ROS2::ComponentStatusCallback(const autoware_system_msgs::msg::ComponentStatus::SharedPtr msg)
{
    component_status_.is_ROSCCO_active = true;
}

void ROS2::ADMADataCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
    gnss_mode_ = msg->data;
}

void ROS2::ROSCCOStatusCallback(const roscco_msgs::msg::RosccoStatus::SharedPtr msg)
{
    roscco_status_.is_brake_enabled = msg->brake_status;
    roscco_status_.is_steer_enabled = msg->steering_status;
    roscco_status_.is_throttle_enabled = msg->throttle_status;
}

void ROS2::ReqAutowareOperationMode(const bool auto_mode)
{
    if(auto_mode)
    {
        std::shared_ptr<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request> request = 
            std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();
        std::shared_future<std::shared_ptr<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Response>> result = 
            AW_auto_client->async_send_request(request);
    }
    else
    {
        std::shared_ptr<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request> request = 
            std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();
        std::shared_future<std::shared_ptr<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Response>> result = 
            AW_stop_client->async_send_request(request);
    }
}

std::pair<float, float> ROS2::updateLocalizationAccuracy()
{
    std::pair<float, float> localization_accuracy;
    localization_accuracy.first = localization_accuracy_long_radius_;
    localization_accuracy.second = localization_accuracy_lateral_direction_;
    return localization_accuracy;
}

void ROS2::pubROSCCOEnableDisable(const bool enable_roscco)
{
    roscco_msgs::msg::EnableDisable msg;
    msg.enable_control = enable_roscco;
    ROSCCO_enable_disable_pub_->publish(msg);
}

ROS2::ComponentStatus ROS2::updateComponentStatus()
{
    return component_status_;
}

ROS2::ROSCCOStatus ROS2::updateROSCCOStatus()
{
    return roscco_status_;
}

int ROS2::updateGNSSMode()
{
    return gnss_mode_;
}