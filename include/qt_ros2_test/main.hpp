#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "roscco_msgs/msg/roscco_status.hpp"
#include "roscco_msgs/msg/enable_disable.hpp"
#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QTimer>
#include <QLabel>
#include <QFrame>

class MainWindow : public QWidget, public rclcpp::Node
{
public:
  MainWindow(QWidget *parent = nullptr);
  std::vector<QFrame *> QFrame_vector;
  std::vector<QLabel *> QLabel_vector;
  std::vector<QPushButton *> QPushButton_vector;
  int label_count;
  int frame_count;
  int btn_count;

private:
  struct Frame_info
  {
    int x;
    int y;
    int width;
    int height;
  };

  struct Label_info
  {
    int x;
    int y;
    int width;
    int height;
    std::string text;
  };

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_trigger_;
  rclcpp::Publisher<roscco_msgs::msg::EnableDisable>::SharedPtr pub_enable_disable_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_localization_accuracy_;
  rclcpp::Subscription<roscco_msgs::msg::RosccoStatus>::SharedPtr sub_roscco_status_;

  QTimer *timer_;

  //btn can use the same structure(Label_info)
  Label_info engage_btn_;
  Label_info estop_btn_;
  Label_info enable_pub_btn_;
  Label_info disable_pub_btn_;

  Frame_info localization_accuracy_frame_;
  Frame_info localization_accuracy_lateral_direction_frame_;
  Frame_info steering_frame_;
  Frame_info brake_frame_;
  Frame_info throttle_frame_;

  Label_info localization_accuracy_label_;
  Label_info localization_accuracy_lateral_direction_label_;
  Label_info roscco_label_;
  Label_info steering_label_;
  Label_info brake_label_;
  Label_info throttle_label_;


  int main_Btn_state;
  float localization_accuracy_;
  float localization_accuracy_lateral_direction_;
  bool brake_enabled;
  bool steering_enabled;
  bool throttle_enabled;
  int roscco_changed;
  int roscco_status;
  int roscco_status_;
  bool estop_enabled_;
  
  std_msgs::msg::Bool trigger_msg;

  void timer_Callback();
  void main_Btn_state_Callback();
  bool Estop_Btn_Callback();
  void enable_pub_Btn_Callback();
  void disable_pub_Btn_Callback();
  void localization_accuracy_Callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void roscco_status_Callback(const roscco_msgs::msg::RosccoStatus::SharedPtr msg);
  void adjust_localization_status(
    const float& localization_accuracy_, const float& localization_accuracy_lateral_direction_);
  void adjust_roscco_status(
    const bool& steering_enabled, const bool& brake_enabled, const bool& throttle_enabled);
  void create_frame(const int& x, const int& y, const int& width, const int& height);
  void create_label(const int& x, const int& y, const int& width,
    const int& height, const std::string& text);
  void create_btn(const int& x, const int& y, const int& width,
    const int& height, const std::string& text);
};
