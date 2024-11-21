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
#include "roscco_msgs/msg/roscco_status.hpp"

class ROS2 : public rclcpp::Node
{
public:
    struct ROSCCOStatus
    {
        bool is_brake_enabled = false;
        bool is_steer_enabled = false;
        bool is_throttle_enabled = false;
    };

    struct SensorStatus
    {
        bool is_Ouster_active{false};
        bool is_ROSCCO_active{false};
        bool is_ADMA_active{false};
        rclcpp::Time current_time;
    };

    explicit ROS2();
    void ReqAutowareOperationMode(const bool auto_mode);
    float* updateLocalizationAccuracy();
    void pubROSCCOEnableDisable(const bool enable_roscco);
    SensorStatus updateSensorStatus();
    ROSCCOStatus updateROSCCOStatus();

private:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_trigger_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr autoware_control_pub_;
    rclcpp::Publisher<roscco_msgs::msg::EnableDisable>::SharedPtr ROSCCO_enable_disable_pub_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr localization_accuracy_sub_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr ouster_clock_sub_;
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr roscco_clock_sub_;
    rclcpp::Subscription<adma_ros_driver_msgs::msg::AdmaDataScaled>::SharedPtr adma_data_sub_;
    rclcpp::Subscription<roscco_msgs::msg::RosccoStatus>::SharedPtr ROSCCO_status_sub_;

    rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr AW_auto_client;
    rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr AW_stop_client;

    rclcpp::TimerBase::SharedPtr timer_;

    float localization_accuracy_ = 0.0;
    float localization_accuracy_lateral_direction_ = 0.0;
    SensorStatus sensor_status_{};
    ROSCCOStatus roscco_status_{};

    void LocalizationAccuracyCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void OusterClockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg);
    void ROSCCOCallback(const std_msgs::msg::Header::SharedPtr msg);
    void ADMADataCallback(const adma_ros_driver_msgs::msg::AdmaDataScaled::SharedPtr msg);
    void ROSCCOStatusCallback(const roscco_msgs::msg::RosccoStatus::SharedPtr msg);
    void TimerCallback();
};

#endif