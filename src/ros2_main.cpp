#include "qt_ros2_gui/ros2_main.hpp"

ROS2::ROS2() : Node("node")
{
    autoware_control_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
        "/vehicle/status/control_mode", rclcpp::QoS(1));
    ROSCCO_enable_disable_pub_ = this->create_publisher<roscco_msgs::msg::EnableDisable>(
        "/enable_disable", rclcpp::QoS(1));

    localization_accuracy_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/localization_accuracy", rclcpp::QoS(1), std::bind(
            &ROS2::LocalizationAccuracyCallback, this, std::placeholders::_1));   
    ouster_clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>
        ("/clock", 10, std::bind(&ROS2::OusterClockCallback, this, std::placeholders::_1));
    roscco_clock_sub_ = this->create_subscription<std_msgs::msg::Header>
        ("/time_from_roscco", 10, std::bind(&ROS2::ROSCCOCallback, this, std::placeholders::_1));
    adma_data_sub_ = this->create_subscription<adma_ros_driver_msgs::msg::AdmaDataScaled>
        ("/genesys/adma/data_scaled", 10, std::bind(&ROS2::ADMADataCallback, this, std::placeholders::_1));

    auto_mode_client = this->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>("/api/operation_mode/change_to_autonomous");

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ROS2::TimerCallback, this));

    localization_accuracy_ = 0;
    localization_accuracy_lateral_direction_ = 0;
    autoware_control_msg.mode = 1;
}

void ROS2::TimerCallback()
{
    autoware_control_pub_->publish(autoware_control_msg);
}

void ROS2::LocalizationAccuracyCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    localization_accuracy_ = std::round(msg->data[0] * 1000.0) / 1000.0;
    localization_accuracy_lateral_direction_ = std::round(msg->data[1] * 1000.0) / 1000.0;
}

void ROS2::OusterClockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
    sensor_status_.current_time = this->now();
    rclcpp::Time Ouster_time = msg->clock;
    const double dt = (sensor_status_.current_time - Ouster_time).seconds();
    if(std::fabs(dt) > 0.1f)
    {
        sensor_status_.isOusterActive = false;
    }
    else
    {
        sensor_status_.isOusterActive = true;
    }
}

void ROS2::ROSCCOCallback(const std_msgs::msg::Header::SharedPtr msg)
{
    sensor_status_.current_time = this->now();
    rclcpp::Time ROSCCO_time = msg->stamp;
    const double dt = (sensor_status_.current_time - ROSCCO_time).seconds();
    if(std::fabs(dt) > 0.1f)
    {
        sensor_status_.isROSCCOActive = false;
    }
    else
    {
        sensor_status_.isROSCCOActive = true;
    }
}

void ROS2::ADMADataCallback(const adma_ros_driver_msgs::msg::AdmaDataScaled::SharedPtr msg)
{
    sensor_status_.current_time = this->now();
    rclcpp::Time AMDA_time = msg->header.stamp;
    const double dt = (sensor_status_.current_time - AMDA_time).seconds();
    if(std::fabs(dt) > 0.1f)
    {
        sensor_status_.isADMAActive = false;
    }
    else
    {
        sensor_status_.isADMAActive = true;
    }
}

void ROS2::SendAutowareModeReq()
{
    std::shared_ptr<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request> request = 
        std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();
    std::shared_future<std::shared_ptr<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Response>> result = 
        auto_mode_client->async_send_request(request);
}

float* ROS2::GetLocalizationAccuracy()
{
    static float localization_status[2];
    localization_status[0] = localization_accuracy_;
    localization_status[1] = localization_accuracy_lateral_direction_;
    return localization_status;
}

void ROS2::pub_roscco_enable_disable(const bool enable_roscco)
{
    roscco_msgs::msg::EnableDisable msg;
    msg.enable_control = enable_roscco;
    ROSCCO_enable_disable_pub_->publish(msg);
}

ROS2::SensorStatus ROS2::UpdateSensorStatus()
{
    return sensor_status_;
}
