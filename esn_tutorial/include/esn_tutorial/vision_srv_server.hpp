#ifndef VISION_SYSTEM_HPP
#define VISION_SYSTEM_HPP
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "esn_msgs/srv/detect_object.hpp"

namespace esn
{

class VisionSystem : public rclcpp::Node
{
protected:
    std::mt19937 gen_;
    std::uniform_real_distribution<double> dist01_;
    std::uniform_real_distribution<double> dist_pos_;
    std::uniform_real_distribution<double> dist_quat_;
    rclcpp::Service<esn_msgs::srv::DetectObject>::SharedPtr service_;

    void cb(const std::shared_ptr<esn_msgs::srv::DetectObject::Request> request,
            std::shared_ptr<esn_msgs::srv::DetectObject::Response> response);
public:
    VisionSystem();

};
}

#endif
