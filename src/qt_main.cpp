#include "qt_ros2_test/qt_main.hpp"

// RCLCPP_INFO(rclcpp::get_logger("test"),"%d", main_Btn_state);

Qtmain::Qtmain(const std::shared_ptr<ROS2>& ros2_node_, QWidget *parent_) : QWidget(parent_), ros2_node(ros2_node_)
{
    label_count = 0;
    frame_count = 0;
    btn_count = 0;

    twist_controller_btn_count = false;
    roscco_enable_btn_Callback_count = false;

    //{pos_x, pos_y, size_x, size_y, text}
    twist_controller_btn_ = {0, 700, 150, 100, "[TwistController] \n Disable Control"};
    auto_mode_btn_ = {150, 700, 150, 100, "[Autoware] \n Auto mode req"};
    enable_pub_btn_ = {300, 700, 150, 100, "[Roscco] \n Enable"};
    disable_pub_btn_ = {450, 700, 150, 100, "[Roscco] \n Disable"};

    localization_accuracy_frame_ = {0, 31, 250, 30};
    localization_accuracy_lateral_direction_frame_ = {0, 61, 250, 30};
    brake_frame_ = {0, 140, 120, 30};
    steering_frame_ = {0, 170, 120, 30};
    throttle_frame_ = {0, 200, 120, 30};
    adma_gnss_mode_frame_ = {0, 280, 150, 30};

    autoware_label_ = {3, 0, 250, 30, "[Autoware]"};
    localization_accuracy_label_ = {3, 31, 250, 30, "localization_accuracy: 0"};
    localization_accuracy_lateral_direction_label_ = {3, 61, 250, 30, "localization_accuracy_LD: 0"};
    roscco_label_ = {3, 110, 200, 30, "[ROSCCO]"};
    brake_label_ = {3, 140, 100, 30, "Brake: 0"};
    steering_label_ = {3, 170, 100, 30, "Steering: 0"};
    throttle_label_ = {3, 200, 100, 30, "Throttle: 0"};
    adma_label_ = {3, 249, 150, 30, "[ADMA]"};
    adma_gnss_mode_label_ = {3, 280, 150, 30, "gnss_mode: 0"};

    this->resize(600, 800);

    QRect screenGeometry = QApplication::desktop()->screenGeometry();
    program_x_ = screenGeometry.width() - this->width();
    program_y_ = 0;
    this->move(program_x_, program_y_);

    timer_ = new QTimer(this);
    timer_->setInterval(20); // TODO_2 : interval speed
    connect(timer_, &QTimer::timeout, this, &Qtmain::timer_Callback);

    //QPushButton_vector[0]
    create_btn(twist_controller_btn_.x, twist_controller_btn_.y, 
                    twist_controller_btn_.width, twist_controller_btn_.height, twist_controller_btn_.text);
    //QPushButton_vector[1]
    create_btn(auto_mode_btn_.x, auto_mode_btn_.y, 
                    auto_mode_btn_.width, auto_mode_btn_.height, auto_mode_btn_.text);
    //QPushButton_vector[2]
    create_btn(enable_pub_btn_.x, enable_pub_btn_.y, 
                    enable_pub_btn_.width, enable_pub_btn_.height, enable_pub_btn_.text);
    //QPushButton_vector[3]
    create_btn(disable_pub_btn_.x, disable_pub_btn_.y, 
                    disable_pub_btn_.width, disable_pub_btn_.height, disable_pub_btn_.text);

    connect(QPushButton_vector[0], &QPushButton::clicked, this, &Qtmain::twist_controller_btn_Callback);
    connect(QPushButton_vector[1], &QPushButton::clicked, this, &Qtmain::auto_mode_btn_Callback);
    connect(QPushButton_vector[2], &QPushButton::clicked, this, &Qtmain::roscco_enable_btn_Callback);
    connect(QPushButton_vector[3], &QPushButton::clicked, this, &Qtmain::roscco_disable_btn_Callback);

    QPushButton_vector[0]->setStyleSheet(
        "background-color: red;" 
        "color: black;"
        "font-family: 'Arial';"
        "font-size: 15px;"
        "font-weight: bold;");

    QPushButton_vector[1]->setStyleSheet(
        "background-color: #efefef;" 
        "color: black;"    
        "font-family: 'Arial';"
        "font-size: 15px;"
        "font-weight: bold;");
    QPushButton_vector[2]->setStyleSheet(
        "background-color: #efefef;" 
        "color: #24b156;"    
        "font-family: 'Arial';"
        "font-size: 15px;"
        "font-weight: bold;");   
    QPushButton_vector[3]->setStyleSheet(
        "background-color: #efefef;" 
        "color: red;"    
        "font-family: 'Arial';"
        "font-size: 15px;"
        "font-weight: bold;");     
        
    //QFrame_vector[0]
    create_frame(localization_accuracy_frame_.x, localization_accuracy_frame_.y, 
                    localization_accuracy_frame_.width, localization_accuracy_frame_.height);
    //QFrame_vector[1]
    create_frame(localization_accuracy_lateral_direction_frame_.x, localization_accuracy_lateral_direction_frame_.y, 
                    localization_accuracy_lateral_direction_frame_.width, localization_accuracy_lateral_direction_frame_.height);
    //QFrame_vector[2]                
    create_frame(brake_frame_.x, brake_frame_.y, 
                    brake_frame_.width, brake_frame_.height);   
    //QFrame_vector[3]                
    create_frame(steering_frame_.x, steering_frame_.y, 
                    steering_frame_.width, steering_frame_.height);
    //QFrame_vector[4]                
    create_frame(throttle_frame_.x, throttle_frame_.y, 
                    throttle_frame_.width, throttle_frame_.height);
    //QFrame_vector[5]                
    create_frame(adma_gnss_mode_frame_.x, adma_gnss_mode_frame_.y, 
                    adma_gnss_mode_frame_.width, adma_gnss_mode_frame_.height);

    //QLabel_vector[0]               
    create_label(localization_accuracy_label_.x, localization_accuracy_label_.y, 
                    localization_accuracy_label_.width, localization_accuracy_label_.height, localization_accuracy_label_.text);
    //QLabel_vector[1]
    create_label(localization_accuracy_lateral_direction_label_.x, localization_accuracy_lateral_direction_label_.y, 
                    localization_accuracy_lateral_direction_label_.width, 
                    localization_accuracy_lateral_direction_label_.height, localization_accuracy_lateral_direction_label_.text);
    //QLabel_vector[2]
    create_label(roscco_label_.x, roscco_label_.y, 
                    roscco_label_.width, roscco_label_.height, roscco_label_.text);
    //QLabel_vector[3]
    create_label(brake_label_.x, brake_label_.y, 
                    brake_label_.width, brake_label_.height, brake_label_.text);    
    //QLabel_vector[4]
    create_label(steering_label_.x, steering_label_.y, 
                    steering_label_.width, steering_label_.height, steering_label_.text);    
    //QLabel_vector[5]
    create_label(throttle_label_.x, throttle_label_.y, 
                    throttle_label_.width, throttle_label_.height, throttle_label_.text);
    //QLabel_vector[6]
    create_label(autoware_label_.x, autoware_label_.y, 
                    autoware_label_.width, autoware_label_.height, autoware_label_.text);
    //QLabel_vector[7]
    create_label(adma_label_.x, adma_label_.y, 
                    adma_label_.width, adma_label_.height, adma_label_.text);
    //QLabel_vector[8]
    create_label(adma_gnss_mode_label_.x, adma_gnss_mode_label_.y, 
                    adma_gnss_mode_label_.width, adma_gnss_mode_label_.height, adma_gnss_mode_label_.text);  
           
    roscco_disable_btn_Callback();
    timer_->start();
}

void Qtmain::twist_controller_btn_Callback()
{
    if(!twist_controller_btn_count)
    {
        QPushButton_vector[0]->setStyleSheet("background-color: #00FF00; color: black;");
        QPushButton_vector[0]->setText("[TwistController] \n Enable Control");
        ros2_node->update_twist_controller_trigger(1);
        twist_controller_btn_count = true;
    }
    else
    {
        QPushButton_vector[0]->setStyleSheet("background-color: red; color: black;");
        QPushButton_vector[0]->setText("[TwistController] \n Disable Control");
        ros2_node->update_twist_controller_trigger(0);
        twist_controller_btn_count = false;
    }
}
void Qtmain::auto_mode_btn_Callback()
{
    ros2_node->send_autoware_mode_req();
}
void Qtmain::roscco_enable_btn_Callback()
{
    ros2_node->pub_roscco_enable();
    roscco_enable_btn_Callback_count = true;
}

void Qtmain::roscco_disable_btn_Callback()
{
    ros2_node->pub_roscco_disable();
}

void Qtmain::timer_Callback()
{   
    float* roscco_cmd = ros2_node->get_roscco_cmd();
    update_roscco_cmd_monitor(roscco_cmd);
    float* localization_accuracy = ros2_node->get_localization_accuracy();
    update_localization_monitor(localization_accuracy);
    const int gnss_mode = ros2_node->get_gnss_mode();
    update_gnss_mode_monitor(gnss_mode);
}

void Qtmain::update_roscco_cmd_monitor(float* roscco_cmd)
{
    QLabel_vector[3]->setText(QString("Brake: %1").arg(roscco_cmd[0]));
    QLabel_vector[4]->setText(QString("Steering: %1").arg(roscco_cmd[1]));
    QLabel_vector[5]->setText(QString("Throttle %1").arg(roscco_cmd[2]));
}

void Qtmain::update_localization_monitor(
    float* localization_accuracy)
{
    if(localization_accuracy[0] > 0.15)
    {
        QFrame_vector[0]->setStyleSheet("background-color: red;");
    }
    else if(localization_accuracy[0] <= 0.15 && localization_accuracy[0] > 0.1)
    {
        QFrame_vector[0]->setStyleSheet("background-color: yellow;");
    }
    else if(localization_accuracy[0] <= 0.1 && localization_accuracy[0] > 0)
    {
        QFrame_vector[0]->setStyleSheet("background-color: #00FF00;");
    }
    else
    {
        QFrame_vector[0]->setStyleSheet("background-color: red;");
    }

    if(localization_accuracy[1] > 0.15)
    {
        QFrame_vector[1]->setStyleSheet("background-color: red;");
    }
    else if(localization_accuracy[1] <= 0.15 && localization_accuracy[1] > 0.1)
    {
        QFrame_vector[1]->setStyleSheet("background-color: yellow;");
    }
    else if(localization_accuracy[1] <= 0.1 && localization_accuracy[1] > 0)
    {
        QFrame_vector[1]->setStyleSheet("background-color: #00FF00;");
    }
    else
    {
        QFrame_vector[1]->setStyleSheet("background-color: red;");
    }

    QString localization_accuracy_text = QString("localization_accuracy: %1").arg(localization_accuracy[0]);
    QLabel_vector[0]->setText(localization_accuracy_text);

    QString localization_accuracy_lateral_direction_text = QString(
        "localization_accuracy_LD: %1").arg(localization_accuracy[1]);
    QLabel_vector[1]->setText(localization_accuracy_lateral_direction_text);
    
    //From Autoware
    /**:
    ros__parameters:
        scale: 3.0
        error_ellipse_size: 1.5
        warn_ellipse_size: 1.2
        error_ellipse_size_lateral_direction: 0.3
        warn_ellipse_size_lateral_direction: 0.25
    **/

}

void Qtmain::update_gnss_mode_monitor(const int gnss_mode)
{
    QString gnss_mode_text;

    switch(gnss_mode)
    {
        case 1:
            gnss_mode_text = "error";
            QFrame_vector[5]->setStyleSheet("background-color: red");
            break;
        case 2:
            gnss_mode_text = "GNSS";
            QFrame_vector[5]->setStyleSheet("background-color: orange");
            break;
        case 4:
            gnss_mode_text = "DGNSS";
            QFrame_vector[5]->setStyleSheet("background-color: yellow");
            break;
        case 8:
            gnss_mode_text = "RTK";
            QFrame_vector[5]->setStyleSheet("background-color: #00FF00");
            break;
        default:
            gnss_mode_text = "error";
            QFrame_vector[5]->setStyleSheet("background-color: red");
            break;           
    }

    QString gnss_mode_text_qt = QString("gnss_mode: %1").arg(gnss_mode_text);
    QLabel_vector[8]->setText(gnss_mode_text_qt);
}

void Qtmain::create_frame(const int& x, const int& y, const int& width, const int& height)
{
    QFrame *newFrame = new QFrame(this);

    newFrame->setGeometry(x, y, width, height);
    newFrame->setLineWidth(2);
    newFrame->setStyleSheet("background-color: #efefef;");
    newFrame->setFrameShape(QFrame::Box);
    
    newFrame->show();
    QFrame_vector.push_back(newFrame);

    frame_count ++;
}

void Qtmain::create_label(const int& x, const int& y, const int& width,
    const int& height, const std::string& text)
{
    QString qText = QString::fromStdString(text);

    QLabel *newLabel = new QLabel(qText, this);

    newLabel->setGeometry(x, y, width, height);
    
    newLabel->show();
    QLabel_vector.push_back(newLabel);

    label_count ++;
}

void Qtmain::create_btn(const int& x, const int& y, const int& width,
    const int& height, const std::string& text)
{
    QString qText = QString::fromStdString(text);

    QPushButton *Btn = new QPushButton(qText, this);

    Btn->setGeometry(x, y, width, height);
    Btn->show();
    QPushButton_vector.push_back(Btn);

    btn_count ++;
}
