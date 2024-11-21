#include "qt_ros2_gui/qt_main.hpp"

// RCLCPP_INFO(rclcpp::get_logger("test"),"%d", main_Btn_state);

Qtmain::Qtmain(const std::shared_ptr<ROS2>& ros2_node_, QWidget *parent_) : QWidget(parent_), ros2_node(ros2_node_)
{
    /** [Adjust size & position of the window]
     *  Currently upper right corner is the default place.
    */
    resize(600, 800);
    QRect screenGeometry = QApplication::desktop()->screenGeometry();
    program_x_ = screenGeometry.width() - width();
    program_y_ = 0;
    move(program_x_, program_y_);




    /** [Create timer]
     * This timer acts the same as ros2 timer.
     * TODO: set appropriate interval speed.
    */
    timer_ = new QTimer(this);
    timer_->setInterval(20); 
    connect(timer_, &QTimer::timeout, this, &Qtmain::TimerCallback);




    /** [Create button]
     * 1. Create button variable with basic setting {pos_x, pos_y, size_x, size_y, text}. see struct Label_info
     * 2. Call create_btn func to initialize button.
     * 3. Call connect func to connect the Callback func.
    */
    AW_auto_btn_ = {0, 700, 150, 100, "[Autoware] \n Auto"};
    AW_stop_btn_ = {150, 700, 150, 100, "[Autoware] \n Stop"};
    ROSCCO_enable_btn_ = {300, 700, 150, 100, "[Roscco] \n Enable"};
    ROSCCO_disable_btn = {450, 700, 150, 100, "[Roscco] \n Disable"};

    //QPushButton_vector[0]
    create_btn(AW_auto_btn_.x, AW_auto_btn_.y, 
                    AW_auto_btn_.width, AW_auto_btn_.height, AW_auto_btn_.text);
    //QPushButton_vector[1]
    create_btn(AW_stop_btn_.x, AW_stop_btn_.y, 
                    AW_stop_btn_.width, AW_stop_btn_.height, AW_stop_btn_.text);
    //QPushButton_vector[2]
    create_btn(ROSCCO_enable_btn_.x, ROSCCO_enable_btn_.y, 
                    ROSCCO_enable_btn_.width, ROSCCO_enable_btn_.height, ROSCCO_enable_btn_.text);
    //QPushButton_vector[3]
    create_btn(ROSCCO_disable_btn.x, ROSCCO_disable_btn.y, 
                    ROSCCO_disable_btn.width, ROSCCO_disable_btn.height, ROSCCO_disable_btn.text);

    connect(QPushButton_vector[0], &QPushButton::clicked, this, &Qtmain::AWAutoBtnCallback);
    connect(QPushButton_vector[1], &QPushButton::clicked, this, &Qtmain::AWStopBtnCallback);
    connect(QPushButton_vector[2], &QPushButton::clicked, this, &Qtmain::ROSCCOEnableBtnCallback);
    connect(QPushButton_vector[3], &QPushButton::clicked, this, &Qtmain::ROSCCODisableBtnCallback);

    QPushButton_vector[0]->setStyleSheet(
        "background-color: #efefef;" 
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




    /** [Create frame]
     * 1. Create frame variable with basic setting {pos_x, pos_y, size_x, size_y}. see struct Frame_info
     * 2. Call create_frame func to initialize frame.
    */
    localization_accuracy_frame_ = {0, 31, 250, 30};
    localization_accuracy_lateral_direction_frame_ = {0, 61, 250, 30};
    brake_frame_ = {0, 140, 120, 30};
    steering_frame_ = {0, 170, 120, 30};
    throttle_frame_ = {0, 200, 120, 30};
    adma_gnss_mode_frame_ = {0, 280, 150, 30};

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




    /** [Create label]
     * 1. Create label variable with basic setting {pos_x, pos_y, size_x, size_y, text}. see struct Label_info
     * 2. Call create_label func to initialize label.
    */ 
    autoware_label_ = {3, 0, 250, 30, "[Autoware]"};
    localization_accuracy_label_ = {3, 31, 250, 30, "localization_accuracy: 0"};
    localization_accuracy_lateral_direction_label_ = {3, 61, 250, 30, "localization_accuracy_LD: 0"};
    roscco_label_ = {3, 110, 200, 30, "[ROSCCO]"};
    brake_label_ = {3, 140, 100, 30, "Brake: 0"};
    steering_label_ = {3, 170, 100, 30, "Steering: 0"};
    throttle_label_ = {3, 200, 100, 30, "Throttle: 0"};
    adma_label_ = {3, 249, 150, 30, "[ADMA]"};
    adma_gnss_mode_label_ = {3, 280, 150, 30, "gnss_mode: 0"};

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




    /** [Start timer]
    */ 
    timer_->start();
}

void Qtmain::AWAutoBtnCallback()
{
    ros2_node->ReqAutowareOperationMode(true);
}

void Qtmain::AWStopBtnCallback()
{
    ros2_node->ReqAutowareOperationMode(false);
}

void Qtmain::ROSCCOEnableBtnCallback()
{
    ros2_node->pub_roscco_enable_disable(true);
}

void Qtmain::ROSCCODisableBtnCallback()
{
    ros2_node->pub_roscco_enable_disable(false);
}

void Qtmain::TimerCallback()
{   
    float* localization_accuracy = ros2_node->GetLocalizationAccuracy();
    update_localization_monitor(localization_accuracy);
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
}

void Qtmain::create_label(const int& x, const int& y, const int& width,
    const int& height, const std::string& text)
{
    QString qText = QString::fromStdString(text);

    QLabel *newLabel = new QLabel(qText, this);

    newLabel->setGeometry(x, y, width, height);
    
    newLabel->show();
    QLabel_vector.push_back(newLabel);
}

void Qtmain::create_btn(const int& x, const int& y, const int& width,
    const int& height, const std::string& text)
{
    QString qText = QString::fromStdString(text);

    QPushButton *Btn = new QPushButton(qText, this);

    Btn->setGeometry(x, y, width, height);
    Btn->show();
    QPushButton_vector.push_back(Btn);
}
