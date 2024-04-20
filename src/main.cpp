#include "qt_ros2_test/main.hpp"

MainWindow::MainWindow(QWidget *parent) : QWidget(parent), Node("node")
{
    Btn1_state = false;
    enable_disable_ = 0;
    localization_accuracy_ = 0;
    localization_accuracy_lateral_direction_ = 0;
    count = 0;

    localization_accuracy_frame_ = {0, 0, 250, 30};
    localization_accuracy_lateral_direction_frame_ = {0, 31, 250, 30};
    enable_disable_frame_ = {0, 80, 180, 30};

    localization_accuracy_label_ = {3, 0, 250, 30, "localization_accuracy: "};
    localization_accuracy_lateral_direction_label_ = {3, 31, 250, 30, "localization_accuracy_LD: "};
    enable_disable_label_ = {3, 80, 200, 30, "enable_disable: "};

    this->resize(300, 500); // TODO_1 : adjust size

    Btn1 = new QPushButton("Stop", this);
    Btn1->setGeometry(0, 400, 150, 100);
    Btn1->setStyleSheet("background-color: red; color: white;");
    connect(Btn1, &QPushButton::clicked, this, &MainWindow::Btn1_Callback);

    Btn2 = new QPushButton("EStop", this);
    Btn2->setGeometry(150, 400, 150, 100);
    Btn2->setStyleSheet("background-color: red; color: white;");
    connect(Btn2, &QPushButton::clicked, this, &MainWindow::Btn2_Callback);

    timer_ = new QTimer(this);
    timer_->setInterval(100); // TODO_2 : interval speed
    connect(timer_, &QTimer::timeout, this, &MainWindow::timer_Callback);

    //QFrame_vector[0]
    create_frame(localization_accuracy_frame_.x, localization_accuracy_frame_.y, 
                    localization_accuracy_frame_.width, localization_accuracy_frame_.height);
    //QFrame_vector[1]
    create_frame(localization_accuracy_lateral_direction_frame_.x, localization_accuracy_lateral_direction_frame_.y, 
                    localization_accuracy_lateral_direction_frame_.width, localization_accuracy_lateral_direction_frame_.height);
    //QFrame_vector[2]                
    create_frame(enable_disable_frame_.x, enable_disable_frame_.y, 
                    enable_disable_frame_.width, enable_disable_frame_.height);

    //QLabel_vector[0]               
    create_label(localization_accuracy_label_.x, localization_accuracy_label_.y, 
                    localization_accuracy_label_.width, localization_accuracy_label_.height, localization_accuracy_label_.text);
    //QLabel_vector[1]
    create_label(localization_accuracy_lateral_direction_label_.x, localization_accuracy_lateral_direction_label_.y, 
                    localization_accuracy_lateral_direction_label_.width, 
                    localization_accuracy_lateral_direction_label_.height, localization_accuracy_lateral_direction_label_.text);
    //QLabel_vector[2]
    create_label(enable_disable_label_.x, enable_disable_label_.y, 
                    enable_disable_label_.width, enable_disable_label_.height, enable_disable_label_.text);

    pub_trigger_ = this->create_publisher<std_msgs::msg::Bool>("/trigger",rclcpp::QoS(1));

    sub_localization_accuracy_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/localization_accuracy", rclcpp::QoS(1), std::bind(
            &MainWindow::localization_accuracy_Callback, this, std::placeholders::_1));
    sub_enable_disable_ = this->create_subscription<std_msgs::msg::Bool>("/enable_disable", rclcpp::QoS(1), std::bind(
        &MainWindow::enable_disable_Callback, this, std::placeholders::_1)); // TODO_4 : topic name, msg type
}

void MainWindow::localization_accuracy_Callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    localization_accuracy_ = msg->data[0];
    localization_accuracy_lateral_direction_ = msg->data[1];
}

void MainWindow::enable_disable_Callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    enable_disable_ = msg->data;
}

void MainWindow::Btn1_Callback()
{
    timer_->start();

    if(Btn1_state == false)
    {
        Btn1->setText("Ready");
        Btn1->setStyleSheet("background-color: yellow; color: black;");
        trigger_msg.data = false;
        Btn1_state = true;
    }
    else if(Btn1_state == true && enable_disable_ == 1) // TODO_6 add more conditions
    {
        Btn1->setText("Start");
        Btn1->setStyleSheet("background-color: #00FF00; color: black;");
        trigger_msg.data = true;
        Btn1_state = false;
    }
    else
    {
        Btn1->setText("Ready");
        Btn1->setStyleSheet("background-color: yellow; color: black;");
        trigger_msg.data = false;
    }
}

void MainWindow::Btn2_Callback()
{
    trigger_msg.data = false;
    // TODO_7 BPS publish?
}

void MainWindow::timer_Callback()
{   
    pub_trigger_->publish(trigger_msg);
    adjust_localization_status(localization_accuracy_, localization_accuracy_lateral_direction_);
    adjust_enable_disable_status(enable_disable_);
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

void MainWindow::adjust_enable_disable_status(const int& enable_disable)
{
    if(enable_disable == false)
    {
        QFrame_vector[2]->setStyleSheet("background-color: red;");
    }
    else
    {
        QFrame_vector[2]->setStyleSheet("background-color: #00FF00;");
    }

    QString text = QString("enable_disable: %1").arg(enable_disable); // TODO_5 : maybe get the feedback from roscco if it is actually enabled
    QLabel_vector[2]->setText(text);
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

    count ++;
    RCLCPP_INFO(rclcpp::get_logger("test"),"create_frame, %p", newFrame);
}

void MainWindow::create_label(const int& x, const int& y, const int& width,
    const int& height, const std::string& text)
{
    QString qText = QString::fromStdString(text);

    QLabel *newLabel = new QLabel(qText, this);

    newLabel->setGeometry(x, y, width, height);
    
    newLabel->show();
    QLabel_vector.push_back(newLabel);
    RCLCPP_INFO(rclcpp::get_logger("test"),"create_label, %p", newLabel);
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
        for(int i = 0; i < 3; i++)
        {
            delete node->QFrame_vector[i];
            delete node->QLabel_vector[i];
        }
        node->QFrame_vector.clear();
        node->QLabel_vector.clear();
    }).detach();
    return app.exec();
}