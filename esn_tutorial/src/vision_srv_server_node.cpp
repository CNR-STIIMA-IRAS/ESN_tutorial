#include <esn_tutorial/vision_srv_server.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<esn::VisionSystem>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
