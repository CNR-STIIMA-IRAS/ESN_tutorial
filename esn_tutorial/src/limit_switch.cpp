#include <esn_tutorial/limit_switch.hpp>

namespace esn{

LimitSwitch::LimitSwitch() : Node("limit_switch_node")
{
    // Leggi i parametri
    this->declare_parameter("topic_name", "object_arrived");
    std::string topic_name = this->get_parameter("topic_name").as_string();

    this->declare_parameter("delta_time", 10.0);
    this->delta_t_ = this->get_parameter("delta_time").as_double();

    this->publisher_ = this->create_publisher<std_msgs::msg::Bool>(topic_name, 10); //10 è la dimensione della coda per la pubblicazione dei messaggi
    this->last_publish_time_ = this->get_clock()->now();

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&LimitSwitch::timer_callback, this)
        );

    RCLCPP_INFO_STREAM(this->get_logger(),"The limit switch will be triggered every "<<this->delta_t_<<" seconds");
}

void LimitSwitch::timer_callback()
{
    rclcpp::Time now = this->get_clock()->now();

    std_msgs::msg::Bool msg;
    if((now - last_publish_time_) > rclcpp::Duration::from_seconds(this->delta_t_))
    {
        msg.data = true;
        last_publish_time_ = now;
    }
    else
    {
        msg.data = false;
    }

    this->publisher_->publish(msg);
}
}
