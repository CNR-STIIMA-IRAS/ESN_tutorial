#ifndef ORCHESTRATOR_HPP
#define ORCHESTRATOR_HPP

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <esn_msgs/action/pick_object.hpp>
#include <esn_msgs/srv/detect_object.hpp>

namespace esn
{

class Orchestrator : public rclcpp::Node
{
public:
    using PickObject      = esn_msgs::action::PickObject;
    using GoalHandle      = rclcpp_action::ClientGoalHandle<PickObject>;
    using DetectObject    = esn_msgs::srv::DetectObject;

    Orchestrator();

private:
    std::string limit_switch_topic_;
    std::string service_name_;
    std::string action_name_;
    std::string object_id_;

    std::atomic_bool busy_;
    std::atomic_bool object_arrived_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr limit_switch_sub_;
    rclcpp::Client<DetectObject>::SharedPtr service_client_;
    rclcpp_action::Client<PickObject>::SharedPtr action_client_;

    void on_limit_switch(const std_msgs::msg::Bool::SharedPtr msg);

    void go_pick_the_object();

    bool send_action_goal(const geometry_msgs::msg::PoseStamped & pose);
    void on_goal_response(GoalHandle::SharedPtr gh);
    void on_feedback(GoalHandle::SharedPtr,
                      const std::shared_ptr<const PickObject::Feedback> feedback);

    void execute();
};

}

#endif
