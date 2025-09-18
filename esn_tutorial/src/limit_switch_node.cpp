#include <esn_tutorial/limit_switch.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<esn::LimitSwitch>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
