#ifndef LIMIT_SWITCH_HPP
#define LIMIT_SWITCH_HPP
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace esn
{

class LimitSwitch : public rclcpp::Node
{
protected:
    double delta_t_;
    rclcpp::Time last_publish_time_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;

    void timer_callback();

public:
    LimitSwitch();

};
}

#endif
