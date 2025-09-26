#include <esn_tutorial/vision_srv_server.hpp>

namespace esn{

VisionSystem::VisionSystem() : Node("vision_srv_server_node"),
    gen_(static_cast<unsigned int>(
        std::chrono::system_clock::now().time_since_epoch().count())),
    dist01_(0.0, 1.0),
    dist_pos_(-5.0, 5.0),
    dist_quat_(-1.0, 1.0)
{

    this->declare_parameter("service_name", "detect_object");
    std::string service_name = this->get_parameter("service_name").as_string();

    this->service_ = this->create_service<esn_msgs::srv::DetectObject>(
        service_name,
        std::bind(&VisionSystem::cb, this, std::placeholders::_1, std::placeholders::_2)
        );
}

void VisionSystem::cb(const std::shared_ptr<esn_msgs::srv::DetectObject::Request> request,
                      std::shared_ptr<esn_msgs::srv::DetectObject::Response> response)
{
    RCLCPP_INFO_STREAM(this->get_logger(),"Received request for detecting object: "<<request->object_id);

    double r = this->dist01_(this->gen_);

    if (r < 0.5)
    {
        RCLCPP_INFO_STREAM(this->get_logger(),"Object "<<request->object_id<<" detected!");

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = rclcpp::Clock().now();
        pose.header.frame_id = "world";

        // Posizione casuale
        pose.pose.position.x = this->dist_pos_(this->gen_);
        pose.pose.position.y = this->dist_pos_(this->gen_);
        pose.pose.position.z = this->dist_pos_(this->gen_);

        // Quaternion casuale
        pose.pose.orientation.x = this->dist_quat_(this->gen_);
        pose.pose.orientation.y = this->dist_quat_(this->gen_);
        pose.pose.orientation.z = this->dist_quat_(this->gen_);
        pose.pose.orientation.w = this->dist_quat_(this->gen_);

        response->pose = pose;
        response->success = true;
    }
    else
    {
        RCLCPP_INFO_STREAM(this->get_logger(),"Object "<<request->object_id<<" NOT detected!");

        response->success = false;
    }
}

}
