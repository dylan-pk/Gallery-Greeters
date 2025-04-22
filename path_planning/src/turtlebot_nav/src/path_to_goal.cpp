#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <sstream>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class PathToGoalClient : public rclcpp::Node {
public:
    PathToGoalClient()
        : Node("path_to_goal_client"), goal_active_(false),
          tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_), running_(true) {
        
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(this, "/compute_path_to_pose");

        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&PathToGoalClient::odometry_callback, this, std::placeholders::_1));

        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("computed_path", 10);

        while (!action_client_->wait_for_action_server(2s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for the compute_path_to_pose action server...");
        }

        rclcpp::sleep_for(2s);
        publish_initial_pose();

        input_thread_ = std::thread(&PathToGoalClient::inputListener, this);
    }

    ~PathToGoalClient() {
        running_ = false;
        cv_.notify_all();
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr action_client_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    nav_msgs::msg::Path computed_path;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::thread input_thread_;
    std::atomic<bool> running_;
    bool goal_active_;
    std::mutex goal_mutex_;
    std::condition_variable cv_;

    void publish_initial_pose() {
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.pose.pose.position.x = -2.0;
        msg.pose.pose.position.y = -0.5;
        msg.pose.pose.position.z = 0.0;
        msg.pose.pose.orientation.z = 0.0;
        msg.pose.pose.orientation.w = 1.0;
        msg.pose.covariance = {
            0.25, 0, 0, 0, 0, 0,
            0, 0.25, 0, 0, 0, 0,
            0, 0, 0.0, 0, 0, 0,
            0, 0, 0, 0.0685, 0, 0,
            0, 0, 0, 0, 0.0685, 0,
            0, 0, 0, 0, 0, 0.0685
        };
        initial_pose_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published initial pose.");
    }

    void send_goal(float x, float y, float theta) {
        auto goal_msg = nav2_msgs::action::ComputePathToPose::Goal();
        goal_msg.goal.header.frame_id = "map";
        goal_msg.goal.header.stamp = this->now();
        goal_msg.use_start = false;

        goal_msg.goal.pose.position.x = x;
        goal_msg.goal.pose.position.y = y;
        goal_msg.goal.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        goal_msg.goal.pose.orientation = tf2::toMsg(q);

        RCLCPP_INFO(this->get_logger(), "Sending goal: (%.2f, %.2f, %.2f)", x, y, theta);

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult & result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(this->get_logger(), "Path computation succeeded. Path contains %lu poses.", result.result->path.poses.size());
                    path_publisher_->publish(result.result->path);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Path computation failed with code %d", static_cast<int>(result.code));
                }
                std::lock_guard<std::mutex> lock(goal_mutex_);
                goal_active_ = false;
                cv_.notify_all();
            };

        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            goal_active_ = true;
        }

        action_client_->async_send_goal(goal_msg, send_goal_options);
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

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Not used directly but available for future use
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathToGoalClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
