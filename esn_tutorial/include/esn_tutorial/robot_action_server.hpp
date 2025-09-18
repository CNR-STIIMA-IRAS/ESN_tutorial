#ifndef ROBOT_ACTION_SERVER_HPP
#define ROBOT_ACTION_SERVER_HPP
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "esn_msgs/action/pick_object.hpp"


namespace esn
{

class RobotAction : public rclcpp::Node
{
protected:
    double initial_x_;
    double initial_y_;
    double motion_duration_;

    rclcpp_action::Server<esn_msgs::action::PickObject>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const esn_msgs::action::PickObject::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<esn_msgs::action::PickObject>> goal_handle);

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<esn_msgs::action::PickObject>> goal_handle);

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<esn_msgs::action::PickObject>> goal_handle);

public:
    RobotAction();

};
}

#endif
