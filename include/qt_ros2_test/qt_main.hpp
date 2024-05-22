#include "qt_ros2_test/ros2_main.hpp"
#include "qt_ros2_test/can_interface.hpp"
#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QTimer>
#include <QLabel>
#include <QFrame>
#include <QDesktopWidget>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>

class Qtmain : public QWidget
{
public:
  explicit Qtmain(const std::shared_ptr<Ros2>& ros2_node, QWidget* parent = nullptr);

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

  const std::shared_ptr<Ros2> ros2_node;
  std::shared_ptr<CAN_Interface> can_obj;
  bool enable_control_btn_count;
  bool twist_controller_btn_count;
  bool roscco_status_prev[3];
  bool roscco_enable_btn_Callback_count;
  int program_x_;
  int program_y_;
  QTimer *timer_;

  //btn can use the same structure(Label_info)
  Label_info twist_controller_btn_;
  Label_info enable_control_btn_;
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
  Label_info steering_status_label_;
  Label_info brake_status_label_;
  Label_info throttle_status_label_;

  void timer_Callback();
  void twist_controller_btn_Callback();
  void enable_control_btn_Callback();
  void roscco_enable_btn_Callback();
  void roscco_disable_btn_Callback();
  void adjust_localization_status(
    const float& localization_accuracy_, const float& localization_accuracy_lateral_direction_);
  void adjust_roscco_status(const bool *roscco_status);
  bool check_roscco_status_change(const bool *roscco_status, const bool roscco_enable_btn_Callback_count);
  void create_frame(const int& x, const int& y, const int& width, const int& height);
  void create_label(const int& x, const int& y, const int& width,
    const int& height, const std::string& text);
  void create_btn(const int& x, const int& y, const int& width,
    const int& height, const std::string& text);
};
