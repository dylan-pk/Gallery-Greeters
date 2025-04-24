#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class SimpleGoalPublisher : public rclcpp::Node {
public:
    SimpleGoalPublisher()
        : Node("simple_goal_publisher") {
        
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_topic", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),  // Send every 5 seconds
            std::bind(&SimpleGoalPublisher::publish_goal, this));
    }

private:
    void publish_goal() {
        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp = this->now();
        goal.header.frame_id = "map";

        goal.pose.position.x = 1.0;
        goal.pose.position.y = 1.0;
        goal.pose.position.z = 0.0;

        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = 0.0;
        goal.pose.orientation.w = 0.0;

        goal_pub_->publish(goal);
        RCLCPP_INFO(this->get_logger(), "Published goal at (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleGoalPublisher>());
    rclcpp::shutdown();
    return 0;
}
