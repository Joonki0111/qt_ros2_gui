#include <QApplication>
#include "qt_ros2_test/qt_main.hpp"
#include "qt_ros2_test/ros2_main.hpp"

static void siginthandler(int /*param*/)
{
    QApplication::quit();
}
int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    rclcpp::init(argc, argv);

    std::shared_ptr<ROS2> ROS2_obj = std::make_shared<ROS2>();
    std::shared_ptr<Qtmain> Qtmain_obj = std::make_shared<Qtmain>(ROS2_obj);

    app.processEvents();
    Qtmain_obj->show();

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(ROS2_obj);

    while (rclcpp::ok())
    {
        exec.spin_some();
        app.processEvents();
    }

    signal(SIGINT, siginthandler);

    exec.remove_node(ROS2_obj);
    rclcpp::shutdown();
}