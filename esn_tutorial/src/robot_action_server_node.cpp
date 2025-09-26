#include <esn_tutorial/robot_action_server.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<esn::RobotAction>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
