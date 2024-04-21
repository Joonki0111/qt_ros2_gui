#include "qt_ros2_test/main.hpp"

// RCLCPP_INFO(rclcpp::get_logger("test"),"%d", main_Btn_state);

MainWindow::MainWindow(QWidget *parent) : QWidget(parent), Node("node")
{
    main_Btn_state = 0;
    steering_enabled = false;
    brake_enabled = false;
    throttle_enabled = false;
    localization_accuracy_ = 0;
    localization_accuracy_lateral_direction_ = 0;
    label_count = 0;
    frame_count = 0;
    btn_count = 0;
    roscco_changed = 0;
    roscco_status = 0;
    roscco_status_ = 0;
    estop_enabled_ = false;

    engage_btn_ = {0, 700, 150, 100, "stop"};
    estop_btn_ = {150, 700, 150, 100, "Estop"};
    enable_pub_btn_ = {300, 700, 150, 100, "pub_enable"};
    disable_pub_btn_ = {450, 700, 150, 100, "pub_disable"};

    localization_accuracy_frame_ = {0, 0, 250, 30};
    localization_accuracy_lateral_direction_frame_ = {0, 31, 250, 30};
    steering_frame_ = {0, 110, 180, 30};
    brake_frame_ = {0, 140, 180, 30};
    throttle_frame_ = {0, 170, 180, 30};

    localization_accuracy_label_ = {3, 0, 250, 30, "localization_accuracy: "};
    localization_accuracy_lateral_direction_label_ = {3, 31, 250, 30, "localization_accuracy_LD: "};
    roscco_label_ = {3, 80, 200, 30, "[roscco]"};
    brake_label_ = {3, 140, 200, 30, "brake: "};
    steering_label_ = {3, 110, 200, 30, "steering: "};
    throttle_label_ = {3, 170, 200, 30, "throttle: "};

    this->resize(600, 800); // TODO_1 : adjust size

    timer_ = new QTimer(this);
    timer_->setInterval(20); // TODO_2 : interval speed
    connect(timer_, &QTimer::timeout, this, &MainWindow::timer_Callback);

    //QPushButton_vector[0]
    create_btn(engage_btn_.x, engage_btn_.y, 
                    engage_btn_.width, engage_btn_.height, engage_btn_.text);
    //QPushButton_vector[1]
    create_btn(estop_btn_.x, estop_btn_.y, 
                    estop_btn_.width, estop_btn_.height, estop_btn_.text);
    //QPushButton_vector[2]
    create_btn(enable_pub_btn_.x, enable_pub_btn_.y, 
                    enable_pub_btn_.width, enable_pub_btn_.height, enable_pub_btn_.text);
    //QPushButton_vector[3]
    create_btn(disable_pub_btn_.x, disable_pub_btn_.y, 
                    disable_pub_btn_.width, disable_pub_btn_.height, disable_pub_btn_.text);

    connect(QPushButton_vector[0], &QPushButton::clicked, this, &MainWindow::main_Btn_state_Callback);
    connect(QPushButton_vector[1], &QPushButton::clicked, this, &MainWindow::Estop_Btn_Callback);
    connect(QPushButton_vector[2], &QPushButton::clicked, this, &MainWindow::enable_pub_Btn_Callback);
    connect(QPushButton_vector[3], &QPushButton::clicked, this, &MainWindow::disable_pub_Btn_Callback);

    QPushButton_vector[2]->setStyleSheet("background-color: #00FF00; color: black;");

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
    create_label(steering_label_.x, steering_label_.y, 
                    steering_label_.width, steering_label_.height, steering_label_.text);    
    //QLabel_vector[4]
    create_label(brake_label_.x, brake_label_.y, 
                    brake_label_.width, brake_label_.height, brake_label_.text);    
    //QLabel_vector[5]
    create_label(throttle_label_.x, throttle_label_.y, 
                    throttle_label_.width, throttle_label_.height, throttle_label_.text);

    pub_trigger_ = this->create_publisher<std_msgs::msg::Bool>("/trigger",rclcpp::QoS(1));
    pub_enable_disable_ = this->create_publisher<roscco_msgs::msg::EnableDisable>("/enable_disable",rclcpp::QoS(1));

    sub_localization_accuracy_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/localization_accuracy", rclcpp::QoS(1), std::bind(
            &MainWindow::localization_accuracy_Callback, this, std::placeholders::_1));

    sub_roscco_status_ = this->create_subscription<roscco_msgs::msg::RosccoStatus>(
        "/roscco_status", rclcpp::QoS(1), std::bind(
            &MainWindow::roscco_status_Callback, this, std::placeholders::_1));
}

void MainWindow::localization_accuracy_Callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    localization_accuracy_ = msg->data[0];
    localization_accuracy_lateral_direction_ = msg->data[1];
}

void MainWindow::roscco_status_Callback(const roscco_msgs::msg::RosccoStatus::SharedPtr msg)
{
    brake_enabled = msg->brake_status;
    steering_enabled = msg->steering_status;
    throttle_enabled = msg->throttle_status;

    roscco_status = brake_enabled + steering_enabled + throttle_enabled;

    RCLCPP_INFO(rclcpp::get_logger("test"),"roscco_status: %d roscco_status_: %d roscco_changed: %d", roscco_status, roscco_status_, roscco_changed);

    if(roscco_status_ != roscco_status)
    {
        roscco_changed ++;
    }

    roscco_status_ = roscco_status;
}

void MainWindow::main_Btn_state_Callback()
{
    roscco_msgs::msg::EnableDisable msg;

    timer_->start();

    if(roscco_status == 0 && main_Btn_state == 0)
    {
        QPushButton_vector[0]->setText("Ready");
        QPushButton_vector[0]->setStyleSheet("background-color: yellow; color: black;");
        msg.enable_control = true;
        pub_enable_disable_->publish(msg);
        trigger_msg.data = false;
        main_Btn_state ++;
    }
    else if(roscco_status > 0 && main_Btn_state == 1)
    {
        QPushButton_vector[0]->setText("start");
        QPushButton_vector[0]->setStyleSheet("background-color: #00FF00; color: black;");
        trigger_msg.data = true;
        main_Btn_state ++;
    }
    else if(main_Btn_state == 2) // TODO_6 add more conditions
    {
        roscco_changed = -1;
        main_Btn_state = -1;
        QPushButton_vector[0]->setText("stop");
        QPushButton_vector[0]->setStyleSheet("background-color: red; color: black;");
        msg.enable_control = false;
        pub_enable_disable_->publish(msg);
        trigger_msg.data = false;
        main_Btn_state ++;
    }
}

bool MainWindow::Estop_Btn_Callback()
{
    if (main_Btn_state == 0 && QPushButton_vector[0]->styleSheet().contains("background-color: red", Qt::CaseInsensitive))
    {
        return false;
    }
    else
    {
        main_Btn_state = 2;
        MainWindow::main_Btn_state_Callback();
        return true;
    }
}

void MainWindow::enable_pub_Btn_Callback()
{
    
    roscco_msgs::msg::EnableDisable msg;
    msg.enable_control = true;
    pub_enable_disable_->publish(msg);
}

void MainWindow::disable_pub_Btn_Callback()
{
    roscco_msgs::msg::EnableDisable msg;
    msg.enable_control = false;
    pub_enable_disable_->publish(msg);
}

void MainWindow::timer_Callback()
{   
    pub_trigger_->publish(trigger_msg);
    if(roscco_changed == 2)
    {
        main_Btn_state = 2;
        MainWindow::main_Btn_state_Callback();
    }
    adjust_localization_status(localization_accuracy_, localization_accuracy_lateral_direction_);
    adjust_roscco_status(steering_enabled, brake_enabled, throttle_enabled);
}

void MainWindow::adjust_localization_status(
    const float& localization_accuracy_, const float& localization_accuracy_lateral_direction_)
{
    if(localization_accuracy_ > 0.2)
    {
        QFrame_vector[0]->setStyleSheet("background-color: red;");
    }
    else if(localization_accuracy_ <= 0.2 && localization_accuracy_ > 0.15)
    {
        QFrame_vector[0]->setStyleSheet("background-color: yellow;");
    }
    else if(localization_accuracy_ <= 0.15 && localization_accuracy_ > 0)
    {
        QFrame_vector[0]->setStyleSheet("background-color: #00FF00;");
    }
    else
    {
        QFrame_vector[0]->setStyleSheet("background-color: red;");
    }

    if(localization_accuracy_lateral_direction_ > 0.2)
    {
        QFrame_vector[1]->setStyleSheet("background-color: red;");
    }
    else if(localization_accuracy_lateral_direction_ <= 0.2 && localization_accuracy_lateral_direction_ > 0.15)
    {
        QFrame_vector[1]->setStyleSheet("background-color: yellow;");
    }
    else if(localization_accuracy_lateral_direction_ <= 0.15 && localization_accuracy_lateral_direction_ > 0)
    {
        QFrame_vector[1]->setStyleSheet("background-color: #00FF00;");
    }
    else
    {
        QFrame_vector[1]->setStyleSheet("background-color: red;");
    }

    QString localization_accuracy_text = QString("localization_accuracy: %1").arg(localization_accuracy_);
    QLabel_vector[0]->setText(localization_accuracy_text);

    QString  localization_accuracy_lateral_direction_text = QString(
        "localization_accuracy_LD: %1").arg(localization_accuracy_lateral_direction_);
    QLabel_vector[1]->setText(localization_accuracy_lateral_direction_text);
}

void MainWindow::adjust_roscco_status(
    const bool& steering_enabled, const bool& brake_enabled, const bool& throttle_enabled)
{
    if(brake_enabled == true)
    {
        QString brake_text = QString("%1 %2").arg(QString::fromStdString(brake_label_.text)).arg("true");
        QFrame_vector[3]->setStyleSheet("background-color: #00FF00;");
        QLabel_vector[4]->setText(brake_text);
    }
    else
    {
        QString brake_text = QString("%1 %2").arg(QString::fromStdString(brake_label_.text)).arg("false");
        QFrame_vector[3]->setStyleSheet("background-color: red;");
        QLabel_vector[4]->setText(brake_text);
    }    

    if(steering_enabled == true)
    {
        QString steering_text = QString("%1 %2").arg(QString::fromStdString(steering_label_.text)).arg("true");
        QFrame_vector[2]->setStyleSheet("background-color: #00FF00;");
        QLabel_vector[3]->setText(steering_text);
    }
    else
    {
        QString steering_text = QString("%1 %2").arg(QString::fromStdString(steering_label_.text)).arg("false");
        QFrame_vector[2]->setStyleSheet("background-color: red;");
        QLabel_vector[3]->setText(steering_text);
    }

    if(throttle_enabled == true)
    {
        QString throttle_text = QString("%1 %2").arg(QString::fromStdString(throttle_label_.text)).arg("true");
        QFrame_vector[4]->setStyleSheet("background-color: #00FF00;");
        QLabel_vector[5]->setText(throttle_text);
    }
    else
    {
        QString throttle_text = QString("%1 %2").arg(QString::fromStdString(throttle_label_.text)).arg("false");
        QFrame_vector[4]->setStyleSheet("background-color: red;");
        QLabel_vector[5]->setText(throttle_text);
    }
}

void MainWindow::create_frame(const int& x, const int& y, const int& width, const int& height)
{
    QFrame *newFrame = new QFrame(this);

    newFrame->setGeometry(x, y, width, height);
    newFrame->setLineWidth(2);
    newFrame->setStyleSheet("background-color: red;");
    newFrame->setFrameShape(QFrame::Box);
    
    newFrame->show();
    QFrame_vector.push_back(newFrame);

    frame_count ++;
}

void MainWindow::create_label(const int& x, const int& y, const int& width,
    const int& height, const std::string& text)
{
    QString qText = QString::fromStdString(text);

    QLabel *newLabel = new QLabel(qText, this);

    newLabel->setGeometry(x, y, width, height);
    
    newLabel->show();
    QLabel_vector.push_back(newLabel);
    label_count ++;
}

void MainWindow::create_btn(const int& x, const int& y, const int& width,
    const int& height, const std::string& text)
{
    QString qText = QString::fromStdString(text);

    QPushButton *Btn = new QPushButton(qText, this);

    Btn->setGeometry(x, y, width, height);
    Btn->setStyleSheet("background-color: red; color: black;");
    
    Btn->show();
    QPushButton_vector.push_back(Btn);
    btn_count ++;
}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    std::shared_ptr<MainWindow> node = std::make_shared<MainWindow>();

    node->show();

    std::thread([&] {
        rclcpp::spin(node);
        rclcpp::shutdown();
        for(int i = 0; i < node->frame_count; i++)
        {
            delete node->QFrame_vector[i];
        }
        for(int i = 0; i < node->label_count; i++)
        {
            delete node->QLabel_vector[i];
        }
        for(int i = 0; i < node->btn_count; i++)
        {
            delete node->QPushButton_vector[i];
        }
        node->QFrame_vector.clear();
        node->QLabel_vector.clear();
        node->QPushButton_vector.clear();
    }).detach();
    return app.exec();
}

