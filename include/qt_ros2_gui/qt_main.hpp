#include "qt_ros2_gui/ros2_main.hpp"
#include <QApplication>
#include <QDesktopWidget>
#include <QPushButton>
#include <QTimer>
#include <QLabel>
#include <QFrame>
#include <QProcess>

class Qtmain : public QWidget
{
public:
  explicit Qtmain(const std::shared_ptr<ROS2>& ros2_node, QWidget* parent = nullptr);
  
  std::vector<QFrame *> QFrame_vector;
  std::vector<QLabel *> QLabel_vector;
  std::vector<QPushButton *> QPushButton_vector;
  
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

  const std::shared_ptr<ROS2> ros2_node;
  int program_x_;
  int program_y_;
  QTimer *timer_;

  //btn can use the same structure(Label_info)
  Label_info auto_mode_btn_;
  Label_info enable_pub_btn_;
  Label_info disable_pub_btn_;

  Frame_info localization_accuracy_frame_;
  Frame_info localization_accuracy_lateral_direction_frame_;
  Frame_info steering_frame_;
  Frame_info brake_frame_;
  Frame_info throttle_frame_;
  Frame_info adma_gnss_mode_frame_;

  Label_info autoware_label_;
  Label_info localization_accuracy_label_;
  Label_info localization_accuracy_lateral_direction_label_;
  Label_info roscco_label_;
  Label_info steering_label_;
  Label_info brake_label_;
  Label_info throttle_label_;
  Label_info adma_label_;
  Label_info adma_gnss_mode_label_;

  void TimerCallback();
  void auto_mode_btn_Callback();
  void roscco_enable_btn_Callback();
  void roscco_disable_btn_Callback();
  void update_roscco_cmd_monitor(float* roscco_cmd);
  void update_localization_monitor(float* localization_accuracy);
  void update_gnss_mode_monitor(const int gnss_mode);
  void create_frame(const int& x, const int& y, const int& width, const int& height);
  void create_label(const int& x, const int& y, const int& width,
    const int& height, const std::string& text);
  void create_btn(const int& x, const int& y, const int& width,
    const int& height, const std::string& text);
};
