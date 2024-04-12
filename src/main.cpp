#include "qt_ros2_test/main.hpp"

MainWindow::MainWindow(QWidget *parent) : QWidget(parent), Node("node")
{
    Btn1_state = false;
    ndt_score_ = 0;
    enable_disable_ = 0;

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

    ndt_frame_ = new QFrame(this);
    ndt_frame_->setFrameShape(QFrame::Box);
    ndt_frame_->setLineWidth(2);
    ndt_frame_->setStyleSheet("background-color: red;");
    ndt_frame_->setGeometry(0, 0, 180, 30);
    
    ndt_label_ = new QLabel("NDT_matching score: ", this);
    ndt_label_->setGeometry(3, 0, 200, 30);

    enable_disable_frame_ = new QFrame(this);
    enable_disable_frame_->setFrameShape(QFrame::Box);
    enable_disable_frame_->setLineWidth(2);
    enable_disable_frame_->setStyleSheet("background-color: red;");
    enable_disable_frame_->setGeometry(0, 40, 180, 30);
    
    enable_disable_label_ = new QLabel("enable_disable: ", this);
    enable_disable_label_->setGeometry(3, 40, 200, 30);

    pub_trigger_ = this->create_publisher<std_msgs::msg::Bool>("/trigger",rclcpp::QoS(1));

    sub_ndt_ = this->create_subscription<std_msgs::msg::Int64>("/ndt_temp",rclcpp::QoS(1),std::bind(
        &MainWindow::NDT_Callback, this, std::placeholders::_1)); // TODO_3 : topic name, msg type
    sub_enable_disable_ = this->create_subscription<std_msgs::msg::Bool>("/enable_disable",rclcpp::QoS(1),std::bind(
        &MainWindow::enable_disable_Callback, this, std::placeholders::_1)); // TODO_4 : topic name, msg type
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
    else if(Btn1_state == true && enable_disable_ == 1) // TODO_5 add more conditions
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
    //BPS publish?
}

void MainWindow::timer_Callback()
{   
    pub_trigger_->publish(trigger_msg);
    const int& ndt_score = check_ndt_score(ndt_score_);
    adjust_ndt_status(ndt_score);
    adjust_enable_disable_status(enable_disable_);
}

void MainWindow::NDT_Callback(const std_msgs::msg::Int64::SharedPtr msg)
{
    ndt_score_ = msg->data;
}

void MainWindow::enable_disable_Callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    enable_disable_ = msg->data;
}

int MainWindow::check_ndt_score(const int& ndt_score_)
{
    if(ndt_score_ <= 1)
    {
        return 0;
    }
    else if(ndt_score_ > 1 && ndt_score_ < 3)
    {
        return 1;
    }
    else
    {
        return 2;
    }
}

void MainWindow::adjust_ndt_status(const int& ndt_score)
{
    if(ndt_score == 0)
    {
        ndt_frame_->setStyleSheet("background-color: red;");
    }
    else if(ndt_score == 1)
    {
        ndt_frame_->setStyleSheet("background-color: yellow;");
    }
    else
    {
        ndt_frame_->setStyleSheet("background-color: #00FF00;");
    }

    QString text = QString("NDT_matching score: %1").arg(ndt_score_);
    ndt_label_->setText(text);
}

void MainWindow::adjust_enable_disable_status(const int& enable_disable)
{
    if(enable_disable == false)
    {
        enable_disable_frame_->setStyleSheet("background-color: red;");
    }
    else
    {
        enable_disable_frame_->setStyleSheet("background-color: #00FF00;");
    }

    QString text = QString("enable_disable: %1").arg(enable_disable); // TODO_5 : maybe get the feedback from roscco if it is actually enabled
    enable_disable_label_->setText(text);
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
    }).detach();

    return app.exec();
}