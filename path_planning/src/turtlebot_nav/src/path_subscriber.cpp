#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"

class PathSubscriber : public rclcpp::Node {
public:
    PathSubscriber() : Node("path_subscriber") {
        // Create a subscription to the "computed_path" topic
        path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
            "computed_path", 10,
            std::bind(&PathSubscriber::path_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "PathSubscriber node has been started.");
    }

private:
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received path with %zu poses", msg->poses.size());
        for (size_t i = 0; i < msg->poses.size(); ++i) {
            const auto &pose = msg->poses[i].pose;
            RCLCPP_INFO(this->get_logger(), "Pose %zu: x=%.2f, y=%.2f, z=%.2f", i, pose.position.x, pose.position.y, pose.position.z);
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}