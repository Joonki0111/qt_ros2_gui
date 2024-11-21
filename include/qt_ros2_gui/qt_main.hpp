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
        Label_info AW_auto_btn_;
        Label_info AW_stop_btn_;
        Label_info ROSCCO_enable_btn_;
        Label_info ROSCCO_disable_btn;

        Frame_info localization_accuracy_frame_;
        Frame_info localization_accuracy_lateral_direction_frame_;
        Frame_info brake_frame_;
        Frame_info steer_frame_;
        Frame_info throttle_frame_;
        Frame_info STS_frame_;
        Frame_info BPS_frame_;
        Frame_info APS_frame_;
        Frame_info adma_gnss_mode_frame_;

        Label_info autoware_label_;
        Label_info localization_accuracy_label_;
        Label_info localization_accuracy_lateral_direction_label_;
        Label_info roscco_label_;
        Label_info brake_label_;
        Label_info steer_label_;
        Label_info throttle_label_;
        Label_info STS_label_;
        Label_info BPS_label_;
        Label_info APS_label_;
        Label_info adma_label_;
        Label_info adma_gnss_mode_label_;

        void TimerCallback();
        void AWAutoBtnCallback();
        void AWStopBtnCallback();
        void ROSCCOEnableBtnCallback();
        void ROSCCODisableBtnCallback();
        void updateRosccoStatusMonitor();
        void updateLocalizationMonitor(float* localization_accuracy);
        void updateGNSSModeMonitor(const int gnss_mode);
        void createFrame(const int& x, const int& y, const int& width, const int& height);
        void createLabel(const int& x, const int& y, const int& width,
            const int& height, const std::string& text);
        void create_btn(const int& x, const int& y, const int& width,
            const int& height, const std::string& text);
};
