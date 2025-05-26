#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <thread>
#include <atomic>
#include <iostream>
#include <sstream>


class SimpleGoalPublisher : public rclcpp::Node {
public:
    SimpleGoalPublisher()
        : Node("simple_goal_publisher"), running_(true) {
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_topic", 10);
        input_thread_ = std::thread(&SimpleGoalPublisher::inputListener, this);
    }

    ~SimpleGoalPublisher() {
        running_ = false;
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    void send_goal(float x, float y, float theta) {
        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp = this->now();
        goal.header.frame_id = "map";
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = 0.0;

        // Simple orientation: only set w for theta=0, otherwise you may want to convert theta to quaternion
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = sin(theta / 2.0);
        goal.pose.orientation.w = cos(theta / 2.0);

        goal_pub_->publish(goal);
        RCLCPP_INFO(this->get_logger(), "Published goal at (%.2f, %.2f, %.2f rad)", x, y, theta);
    }

    void inputListener() {
        std::string input;
        while (running_) {
            std::cout << "Enter goal (x y theta in radians): ";
            std::getline(std::cin, input);
            std::istringstream iss(input);
            float x, y, theta;
            if (iss >> x >> y >> theta) {
                send_goal(x, y, theta);
            } else {
                std::cout << "Invalid input. Please enter: x y theta\n";
            }
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    std::thread input_thread_;
    std::atomic<bool> running_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleGoalPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
