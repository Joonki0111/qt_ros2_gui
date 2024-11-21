#ifndef QT_ROS2_GUI__ROS2_MAIN_HPP_
#define QT_ROS2_GUI__ROS2_MAIN_HPP_

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "autoware_auto_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_adapi_v1_msgs/srv/change_operation_mode.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_msgs/msg/header.hpp"
#include "adma_ros_driver_msgs/msg/adma_data_scaled.hpp"
#include "roscco_msgs/msg/enable_disable.hpp"

class ROS2 : public rclcpp::Node
{
public:
    explicit ROS2();
    void SendAutowareModeReq();
    float* GetLocalizationAccuracy();
    void pub_roscco_enable_disable(const bool enable_roscco);

private:
    struct SensorStatus
    {
        bool isOusterActive{false};
        bool isROSCCOActive{false};
        bool isADMAActive{false};
        rclcpp::Time current_time;
    };

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_trigger_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr autoware_control_pub_;
    rclcpp::Publisher<roscco_msgs::msg::EnableDisable>::SharedPtr ROSCCO_enable_disable_pub_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr localization_accuracy_sub_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr ouster_clock_sub_;
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr roscco_clock_sub_;
    rclcpp::Subscription<adma_ros_driver_msgs::msg::AdmaDataScaled>::SharedPtr adma_data_sub_;

    rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr auto_mode_client;

    rclcpp::TimerBase::SharedPtr timer_;

    autoware_auto_vehicle_msgs::msg::ControlModeReport autoware_control_msg;

    float localization_accuracy_;
    float localization_accuracy_lateral_direction_;

    SensorStatus sensor_status_{};

    void LocalizationAccuracyCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void OusterClockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg);
    void ROSCCOCallback(const std_msgs::msg::Header::SharedPtr msg);
    void ADMADataCallback(const adma_ros_driver_msgs::msg::AdmaDataScaled::SharedPtr msg);
    void TimerCallback();
    SensorStatus UpdateSensorStatus();

};

#endif