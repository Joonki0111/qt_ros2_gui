#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/bool.hpp"
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

private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_trigger_;

  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_ndt_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_disable_;

  QPushButton *Btn1;
  QPushButton *Btn2;
  QTimer *timer_;
  QFrame *ndt_frame_;
  QLabel *ndt_label_;
  QFrame *enable_disable_frame_;
  QLabel *enable_disable_label_;

  bool Btn1_state;
  int test_num;
  int ndt_score_;
  bool enable_disable_;

  std_msgs::msg::Bool trigger_msg;

  void timer_Callback();
  void Btn1_Callback();
  void Btn2_Callback();
  void NDT_Callback(const std_msgs::msg::Int64::SharedPtr msg);
  void enable_disable_Callback(const std_msgs::msg::Bool::SharedPtr msg);
  int check_ndt_score(const int& ndt_score_);
  void adjust_ndt_status(const int& ndt_score);
  void adjust_enable_disable_status(const int& enable_disable);
};
