#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/bool.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
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

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_localization_accuracy_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_disable_;

  QPushButton *Btn1;
  QPushButton *Btn2;
  QTimer *timer_;

  Label_info localization_accuracy_label_;
  Label_info localization_accuracy_lateral_direction_label_;
  Label_info enable_disable_label_;

  Frame_info localization_accuracy_frame_;
  Frame_info localization_accuracy_lateral_direction_frame_;
  Frame_info enable_disable_frame_;

  bool Btn1_state;
  float localization_accuracy_;
  float localization_accuracy_lateral_direction_;
  bool enable_disable_;
  int count;

  std_msgs::msg::Bool trigger_msg;

  void timer_Callback();
  void Btn1_Callback();
  void Btn2_Callback();
  void localization_accuracy_Callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void enable_disable_Callback(const std_msgs::msg::Bool::SharedPtr msg);
  void adjust_localization_status(
    const float& localization_accuracy_, const float& localization_accuracy_lateral_direction_);
  void adjust_enable_disable_status(const int& enable_disable);
  void create_frame(const int& x, const int& y, const int& width, const int& height);
  void create_label(const int& x, const int& y, const int& width,
    const int& height, const std::string& text);
};
