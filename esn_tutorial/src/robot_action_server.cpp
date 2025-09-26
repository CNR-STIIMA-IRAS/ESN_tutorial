#include <esn_tutorial/robot_action_server.hpp>
#include <functional>
#include <thread>

namespace esn {

RobotAction::RobotAction() : Node("robot_action_node")
{
    this->declare_parameter("motion_duration", 2.0);
    this->motion_duration_ = this->get_parameter("motion_duration").as_double();

    this->declare_parameter("initial_x", 0.0);
    this->initial_x_ = this->get_parameter("initial_x").as_double();

    this->declare_parameter("initial_y", 0.0);
    this->initial_y_ = this->get_parameter("initial_y").as_double();

    this->declare_parameter("action_name", "take_object");
    std::string action_name = this->get_parameter("action_name").as_string();

    this->cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    this->action_server_ = rclcpp_action::create_server<esn_msgs::action::PickObject>(
        this,
        action_name,
        std::bind(&RobotAction::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&RobotAction::handle_cancel, this, std::placeholders::_1),
        std::bind(&RobotAction::handle_accepted, this, std::placeholders::_1)
        );
}

rclcpp_action::GoalResponse RobotAction::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const esn_msgs::action::PickObject::Goal> goal)
{
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Received goal pose [" << goal->pose.header.frame_id << "] "
                               << "pos=(" << goal->pose.pose.position.x << ", "
                               << goal->pose.pose.position.y << ")"
        );
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotAction::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<esn_msgs::action::PickObject>> /*goal_handle*/)
{
    RCLCPP_INFO(this->get_logger(), "Cancel request received");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotAction::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<esn_msgs::action::PickObject>> goal_handle)
{
    std::thread(&RobotAction::execute, this, goal_handle).detach();
}

void RobotAction::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<esn_msgs::action::PickObject>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal...");

    const auto goal = goal_handle->get_goal();

    const double target_x = goal->pose.pose.position.x;
    const double target_y = goal->pose.pose.position.y;

    const double dx = target_x - this->initial_x_;
    const double dy = target_y - this->initial_y_;

    const double T = this->motion_duration_ > 1e-6 ? this->motion_duration_ : 1e-6;
    const double vx = dx / T;
    const double vy = dy / T;

    RCLCPP_INFO(this->get_logger(),
                "Computed velocity: vx=%.3f, vy=%.3f to reach in %.2fs", vx, vy, T);

    const double rate_hz = 10.0;
    rclcpp::Rate rate(rate_hz);
    const double dt = 1.0 / rate_hz;

    double t = 0.0;
    geometry_msgs::msg::Twist cmd;
    geometry_msgs::msg::Twist stop;  // zero cmd

    auto feedback = std::make_shared<esn_msgs::action::PickObject::Feedback>();
    auto result   = std::make_shared<esn_msgs::action::PickObject::Result>();

    while (t < T && rclcpp::ok())
    {
        if (goal_handle->is_canceling())
        {
            this->cmd_vel_pub_->publish(stop);

            feedback->progress = static_cast<float>(t / T);
            goal_handle->publish_feedback(feedback);

            result->success = false;
            goal_handle->canceled(result);

            RCLCPP_WARN(this->get_logger(),
                        "Goal canceled at %.0f%%", feedback->progress * 100.0);
            return;
        }

        cmd.linear.x = vx;
        cmd.linear.y = vy;
        this->cmd_vel_pub_->publish(cmd);

        feedback->progress = static_cast<float>(t / T);
        goal_handle->publish_feedback(feedback);

        rate.sleep();
        t += dt;
    }

    this->cmd_vel_pub_->publish(stop);

    if (rclcpp::ok())
    {
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(),
                    "Goal succeeded, reached target pose");

        this->initial_x_ = target_x;
        this->initial_y_ = target_y;
    }
}

}
