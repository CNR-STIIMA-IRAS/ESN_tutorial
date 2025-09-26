#include <esn_tutorial/orchestrator.hpp>

namespace esn
{

Orchestrator::Orchestrator() : rclcpp::Node("orchestrator_node")
{
    this->declare_parameter<std::string>("topic_name", "object_arrived");
    this->limit_switch_topic_ = this->get_parameter("topic_name").as_string();

    this->declare_parameter<std::string>("service_name", "detect_object");
    this->service_name_ = this->get_parameter("service_name").as_string();

    this->declare_parameter<std::string>("action_name", "take_object");
    this->action_name_ = this->get_parameter("action_name").as_string();

    this->declare_parameter<std::string>("object_id", "target_1");
    this->object_id_ = this->get_parameter("object_id").as_string();

    this->limit_switch_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        this->limit_switch_topic_, 10,
        std::bind(&Orchestrator::on_limit_switch, this, std::placeholders::_1));

    this->service_client_ = this->create_client<DetectObject>(this->service_name_);

    this->action_client_ = rclcpp_action::create_client<PickObject>(this, this->action_name_);

    this->busy_ = false;
    this->object_arrived_ = false;

    RCLCPP_INFO(this->get_logger(),
                "Orchestrator ready. Subscribing '%s', service '%s', action '%s'",
                this->limit_switch_topic_.c_str(),
                this->service_name_.c_str(),
                this->action_name_.c_str());

    std::thread(&Orchestrator::execute, this).detach();
}

void Orchestrator::on_limit_switch(const std_msgs::msg::Bool::SharedPtr msg)
{
    this->object_arrived_ = msg->data;
}

void Orchestrator::go_pick_the_object()
{
    this->busy_ = true;

    geometry_msgs::msg::PoseStamped pose;

    if (!this->service_client_->wait_for_service(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(this->get_logger(), "Service '%s' not available", this->service_name_.c_str());
        this->busy_ = false;
        return;
    }

    auto req = std::make_shared<DetectObject::Request>();
    req->object_id = this->object_id_;

    auto future = this->service_client_->async_send_request(req);

    future.wait();

    auto resp = future.get();
    if (resp->success)
    {
        RCLCPP_INFO(this->get_logger(), "VisionSystem: object detected");
        this->send_action_goal(resp->pose);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "VisionSystem: object NOT detected");
    }

    this->busy_ = false;

    return;
}

bool Orchestrator::send_action_goal(const geometry_msgs::msg::PoseStamped & pose)
{
    using rclcpp_action::ResultCode;

    if (!this->action_client_->wait_for_action_server(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(this->get_logger(), "Action server '%s' not available", this->action_name_.c_str());
        return false;
    }

    PickObject::Goal goal;
    goal.pose = pose;

    rclcpp_action::Client<PickObject>::SendGoalOptions opts;
    opts.feedback_callback      = std::bind(&Orchestrator::on_feedback, this,
                                       std::placeholders::_1, std::placeholders::_2);

    // (opts.goal_response_callback non serve: la risposta al goal viene gestita attendendo gh_future.wait())
    // (opts.result_callback non serve: aspettiamo il risultato con .wait())

    auto gh_future = this->action_client_->async_send_goal(goal, opts);
    gh_future.wait();

    auto gh = gh_future.get();
    if (!gh) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by action server");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Goal accepted");

    auto result_future = this->action_client_->async_get_result(gh);
    result_future.wait();

    auto result = result_future.get();
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Action SUCCEEDED, success=%s",
                    result.result->success ? "true" : "false");
        return result.result->success;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(this->get_logger(), "Action ABORTED");
        return false;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Action CANCELED");
        return false;
    default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return false;
    }
}

void Orchestrator::on_feedback(GoalHandle::SharedPtr,
                               const std::shared_ptr<const PickObject::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(), "Progress: %.0f%%", feedback->progress * 100.0);
}

void Orchestrator::execute()
{
    rclcpp::Rate rate(10.0);
    while (rclcpp::ok())
    {
        if (!this->busy_ && this->object_arrived_)
        {
            RCLCPP_INFO(this->get_logger(),"-----------------------------");
            RCLCPP_INFO(this->get_logger(),"New object arrived");
            this->go_pick_the_object();
        }
        rate.sleep();
    }

    RCLCPP_INFO(this->get_logger(),"Stopping the orchestrator...");
}


} // namespace esn
