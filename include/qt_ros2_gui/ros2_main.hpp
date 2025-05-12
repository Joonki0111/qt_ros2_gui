#ifndef QT_ROS2_GUI__ROS2_MAIN_HPP_
#define QT_ROS2_GUI__ROS2_MAIN_HPP_

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"
#include "autoware_localization_msgs/msg/localization_accuracy.hpp"
#include "autoware_adapi_v1_msgs/srv/change_operation_mode.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/int8.hpp"
#include "roscco_msgs/msg/enable_disable.hpp"
#include "roscco_msgs/msg/roscco_status.hpp"
#include "autoware_system_msgs/msg/component_status.hpp"

class ROS2 : public rclcpp::Node
{
public:
    struct ROSCCOStatus
    {
        bool is_brake_enabled = false;
        bool is_steer_enabled = false;
        bool is_throttle_enabled = false;
    };

    struct ComponentStatus
    {
        bool is_Ouster_active{false};
        bool is_ROSCCO_active{false};
        bool is_ADMA_active{false};
        bool is_TC_active{false};
        bool is_ROSCCO_CAN_active{false};
        bool is_vehicle_CAN_active{false};
    };

    int gnss_mode_ = 0;

    explicit ROS2();
    bool ReqAutowareOperationMode(const bool auto_mode);
    std::pair<float, float> updateLocalizationAccuracy();
    void pubROSCCOEnableDisable(const bool enable_roscco);
    ComponentStatus updateComponentStatus();
    ROSCCOStatus updateROSCCOStatus();
    int updateGNSSMode();

private:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_trigger_;
    rclcpp::Publisher<roscco_msgs::msg::EnableDisable>::SharedPtr ROSCCO_enable_disable_pub_;

    rclcpp::Subscription<autoware_localization_msgs::msg::LocalizationAccuracy>::SharedPtr localization_accuracy_sub_;
    rclcpp::Subscription<autoware_system_msgs::msg::ComponentStatus>::SharedPtr component_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr adma_gnss_mode_sub_;
    rclcpp::Subscription<roscco_msgs::msg::RosccoStatus>::SharedPtr ROSCCO_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr steer_aligned_status_sub_;

    rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr AW_auto_client;
    rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr AW_stop_client;

    rclcpp::TimerBase::SharedPtr timer_;

    float localization_accuracy_long_radius_ = 0.0;
    float localization_accuracy_lateral_direction_ = 0.0;
    ComponentStatus component_status_{};
    ROSCCOStatus roscco_status_{};
    bool is_steer_aligned_ = false;

    void TimerCallback();
    void LocalizationAccuracyCallback(const autoware_localization_msgs::msg::LocalizationAccuracy::SharedPtr msg);
    void ComponentStatusCallback(const autoware_system_msgs::msg::ComponentStatus::SharedPtr msg);
    void SteerAlignedStatusCallback(const std_msgs::msg::Bool msg);
    void ADMADataCallback(const std_msgs::msg::Int8::SharedPtr msg);
    void ROSCCOStatusCallback(const roscco_msgs::msg::RosccoStatus::SharedPtr msg);
};

#endif