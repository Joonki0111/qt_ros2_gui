#include "qt_ros2_gui/qt_main.hpp"

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
    timer_->setInterval(100); 
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

    create_btn(AW_auto_btn_); //QPushButton_vector[0]
    create_btn(AW_stop_btn_); //QPushButton_vector[1]
    create_btn(ROSCCO_enable_btn_); //QPushButton_vector[2]
    create_btn(ROSCCO_disable_btn); //QPushButton_vector[3]

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
     * 2. Call createFrame func to initialize frame.
    */
    localization_accuracy_frame_ = {0, 31, 250, 30};
    localization_accuracy_lateral_direction_frame_ = {0, 61, 250, 30};
    brake_frame_ = {0, 140, 140, 30};
    steer_frame_ = {0, 170, 140, 30};
    throttle_frame_ = {0, 200, 140, 30};
    BPS_frame_ = {150, 140, 70, 30};
    STS_frame_ = {150, 170, 70, 30};
    APS_frame_ = {150, 200, 70, 30};
    adma_gnss_mode_frame_ = {0, 280, 150, 30};

    createFrame(localization_accuracy_frame_); //QFrame_vector[0]
    createFrame(localization_accuracy_lateral_direction_frame_); //QFrame_vector[1]
    createFrame(brake_frame_); //QFrame_vector[2]
    createFrame(steer_frame_); //QFrame_vector[3]
    createFrame(throttle_frame_); //QFrame_vector[4]
    createFrame(BPS_frame_); //QFrame_vector[5]
    createFrame(STS_frame_); //QFrame_vector[6]
    createFrame(APS_frame_); //QFrame_vector[7]
    createFrame(adma_gnss_mode_frame_); //QFrame_vector[8] 

    /** [Create label]
     * 1. Create label variable with basic setting {pos_x, pos_y, size_x, size_y, text}. see struct Label_info
     * 2. Call createLabel func to initialize label.
    */ 
    autoware_label_ = {3, 0, 250, 30, "[Autoware]"};
    localization_accuracy_label_ = {3, 31, 250, 30, "localization_accuracy: 0"};
    localization_accuracy_lateral_direction_label_ = {3, 61, 250, 30, "localization_accuracy_LD: 0"};
    ROSCCO_label_ = {3, 110, 180, 30, "[ROSCCO]     Off"};
    brake_label_ = {3, 140, 130, 30, "Brake Disabled"};
    steer_label_ = {3, 170, 130, 30, "Steer Disabled"};
    throttle_label_ = {3, 200, 130, 30, "Throttle Disabled"};
    BPS_label_ = {153, 140, 100, 30, "BPS: 0"};
    STS_label_ = {153, 170, 100, 30, "STS: 0"};
    APS_label_ = {153, 200, 100, 30, "APS: 0"};
    ADMA_label_ = {3, 249, 180, 30, "[ADMA]     Off"};
    ADMA_GNSS_mode_label_ = {3, 280, 150, 30, "gnss_mode: 0"};
    Ouster_label_ = {3, 330, 180, 30, "[Ouster]     Off"};
    TC_status_label_ = {3, 400, 180, 30, "[TwistController]     Off"};
    ROSCCO_CAN_status_label_ = {3, 500, 150, 30, "[ROSCCO CAN]     Off"};
    vehicle_CAN_status_label_ = {3, 600, 150, 30, "[vehicle CAN]     Off"};

    createLabel(autoware_label_); //QLabel_vector[0]
    createLabel(localization_accuracy_label_); //QLabel_vector[1]
    createLabel(localization_accuracy_lateral_direction_label_); //QLabel_vector[2]
    createLabel(ROSCCO_label_); //QLabel_vector[3]
    createLabel(brake_label_); //QLabel_vector[4]
    createLabel(steer_label_); //QLabel_vector[5]
    createLabel(throttle_label_); //QLabel_vector[6]
    createLabel(BPS_label_); //QLabel_vector[7]
    createLabel(STS_label_); //QLabel_vector[8]
    createLabel(APS_label_); //QLabel_vector[9]
    createLabel(ADMA_label_); //QLabel_vector[10]
    createLabel(ADMA_GNSS_mode_label_); //QLabel_vector[11]
    createLabel(Ouster_label_); //QLabel_vector[12]
    createLabel(TC_status_label_); //QLabel_vector[13]
    createLabel(ROSCCO_CAN_status_label_); //QLabel_vector[14]
    createLabel(vehicle_CAN_status_label_); //QLabel_vector[15]
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
    ros2_node->pubROSCCOEnableDisable(true);
}

void Qtmain::ROSCCODisableBtnCallback()
{
    ros2_node->pubROSCCOEnableDisable(false);
}

void Qtmain::TimerCallback()
{   
    const std::pair<float, float> localization_accuracy = ros2_node->updateLocalizationAccuracy();
    updateLocalizationMonitor(localization_accuracy);
    updateRosccoStatusMonitor();
    updateComponentStatusMonitor();
    const int gnss_mode = ros2_node->updateGNSSMode();
    updateGNSSModeMonitor(gnss_mode);
}

void Qtmain::updateRosccoStatusMonitor()
{
    ROS2::ROSCCOStatus roscco_status = ros2_node->updateROSCCOStatus();

    if(roscco_status.is_brake_enabled)
    {
        QLabel_vector[4]->setText(QString("Brake Enabled"));
        QFrame_vector[2]->setStyleSheet("background-color: #00FF00;");
    }
    else
    {
        QLabel_vector[4]->setText(QString("Brake Disabled"));
        QFrame_vector[2]->setStyleSheet("background-color: red;");
    }

    if(roscco_status.is_steer_enabled)
    {
        QLabel_vector[5]->setText(QString("Steer Enabled"));
        QFrame_vector[3]->setStyleSheet("background-color: #00FF00;");
    }
    else
    {
        QLabel_vector[5]->setText(QString("Steer Disabled"));
        QFrame_vector[3]->setStyleSheet("background-color: red;");
    }
        
    if(roscco_status.is_throttle_enabled)
    {
        QLabel_vector[6]->setText(QString("Throttle Enabled"));
        QFrame_vector[4]->setStyleSheet("background-color: #00FF00;");
    }
    else
    {
        QLabel_vector[6]->setText(QString("Throttle Disabled"));
        QFrame_vector[4]->setStyleSheet("background-color: red;");
    }
}

void Qtmain::updateLocalizationMonitor(const std::pair<float, float> localization_accuracy)
{
    if(localization_accuracy.first > 0.15f)
    {
        QFrame_vector[0]->setStyleSheet("background-color: red;");
    }
    else if(localization_accuracy.first <= 0.15f && localization_accuracy.first > 0.1f)
    {
        QFrame_vector[0]->setStyleSheet("background-color: yellow;");
    }
    else if(localization_accuracy.first <= 0.1f && localization_accuracy.first > 0.0)
    {
        QFrame_vector[0]->setStyleSheet("background-color: #00FF00;");
    }
    else
    {
        QFrame_vector[0]->setStyleSheet("background-color: red;");
    }

    if(localization_accuracy.second > 0.15f)
    {
        QFrame_vector[1]->setStyleSheet("background-color: red;");
    }
    else if(localization_accuracy.second <= 0.15f && localization_accuracy.second > 0.1f)
    {
        QFrame_vector[1]->setStyleSheet("background-color: yellow;");
    }
    else if(localization_accuracy.second <= 0.1f && localization_accuracy.second > 0.0)
    {
        QFrame_vector[1]->setStyleSheet("background-color: #00FF00;");
    }
    else
    {
        QFrame_vector[1]->setStyleSheet("background-color: red;");
    }

    QString localization_accuracy_text = QString("localization_accuracy: %1").arg(localization_accuracy.first);
    QLabel_vector[1]->setText(localization_accuracy_text);

    QString localization_accuracy_lateral_direction_text = QString(
        "localization_accuracy_LD: %1").arg(localization_accuracy.second);
    QLabel_vector[2]->setText(localization_accuracy_lateral_direction_text);
}

void Qtmain::updateGNSSModeMonitor(const int gnss_mode)
{
    QString gnss_mode_text;

    switch(gnss_mode)
    {
        case 1:
            gnss_mode_text = "error";
            QFrame_vector[8]->setStyleSheet("background-color: red");
            break;
        case 2:
            gnss_mode_text = "GNSS";
            QFrame_vector[8]->setStyleSheet("background-color: orange");
            break;
        case 4:
            gnss_mode_text = "DGNSS";
            QFrame_vector[8]->setStyleSheet("background-color: yellow");
            break;
        case 8:
            gnss_mode_text = "RTK";
            QFrame_vector[8]->setStyleSheet("background-color: #00FF00");
            break;
        default:
            gnss_mode_text = "error";
            QFrame_vector[8]->setStyleSheet("background-color: red");
            break;           
    }

    QString gnss_mode_text_qt = QString("gnss_mode: %1").arg(gnss_mode_text);
    QLabel_vector[11]->setText(gnss_mode_text_qt);
}

void Qtmain::updateComponentStatusMonitor()
{
    ROS2::ComponentStatus component_status = ros2_node->updateComponentStatus();
    
    // ROSCCO status
    if (component_status.is_ROSCCO_active) QLabel_vector[3]->setText(QString("<font color='green'>[ROSCCO]     On</font>"));
    else QLabel_vector[3]->setText(QString("<font color='red'>[ROSCCO]     Off</font>"));

    // ADMA status
    if (component_status.is_ADMA_active) QLabel_vector[10]->setText(QString("<font color='green'>[ADMA]     On</font>"));
    else QLabel_vector[10]->setText(QString("<font color='red'>[ADMA]     Off</font>"));

    // Ouster status
    if (component_status.is_Ouster_active) QLabel_vector[12]->setText(QString("<font color='green'>[Ouster]     On</font>"));
    else QLabel_vector[12]->setText(QString("<font color='red'>[Ouster]     Off</font>"));

    // TwistController status
    if (component_status.is_TC_active) QLabel_vector[13]->setText(QString("<font color='green'>[TwistController]     On</font>"));
    else QLabel_vector[13]->setText(QString("<font color='red'>[TwistController]     Off</font>"));

    // ROSCCO CAN status
    if (component_status.is_ROSCCO_CAN_active) QLabel_vector[14]->setText(QString("<font color='green'>[ROSCCO CAN]     On</font>"));
    else QLabel_vector[14]->setText(QString("<font color='red'>[ROSCCO CAN]     Off</font>"));

    // Vehicle CAN status
    if (component_status.is_vehicle_CAN_active) QLabel_vector[15]->setText(QString("<font color='green'>[vehicle CAN]     On</font>"));
    else QLabel_vector[15]->setText(QString("<font color='red'>[vehicle CAN]     Off</font>"));
}

void Qtmain::createFrame(const Frame_info frame_info)
{
    QFrame *newFrame = new QFrame(this);

    newFrame->setGeometry(frame_info.x, frame_info.y, frame_info.width, frame_info.height);
    newFrame->setLineWidth(2);
    newFrame->setStyleSheet("background-color: #efefef;");
    newFrame->setFrameShape(QFrame::Box);
    
    newFrame->show();
    QFrame_vector.push_back(newFrame);
}

void Qtmain::createLabel(const Label_info label_info)
{
    QString qText = QString::fromStdString(label_info.text);

    QLabel *newLabel = new QLabel(qText, this);

    newLabel->setGeometry(label_info.x, label_info.y, label_info.width, label_info.height);
    
    newLabel->show();
    QLabel_vector.push_back(newLabel);
}

void Qtmain::create_btn(const Label_info btn_info)
{
    QString qText = QString::fromStdString(btn_info.text);

    QPushButton *Btn = new QPushButton(qText, this);

    Btn->setGeometry(btn_info.x, btn_info.y, btn_info.width, btn_info.height);
    Btn->show();
    QPushButton_vector.push_back(Btn);
}
