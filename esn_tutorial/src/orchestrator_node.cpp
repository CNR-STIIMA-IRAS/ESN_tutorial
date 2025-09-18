#include <esn_tutorial/orchestrator.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<esn::Orchestrator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
