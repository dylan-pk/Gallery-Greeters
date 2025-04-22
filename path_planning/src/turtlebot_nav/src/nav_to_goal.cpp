#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <condition_variable>  // Add this include for condition_variable
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class TurtlebotNavNode : public rclcpp::Node {
public:
    TurtlebotNavNode()
        : Node("turtlebot_nav_node"), goal_active_(false),
          tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_), running_(true) {
        
        // Create an action client to send navigation goals
        this->action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

        // Create a subscription to the odometry topic
        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&TurtlebotNavNode::odometry_callback, this, std::placeholders::_1));

        // Wait until the action server is available
        while (!this->action_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for the navigate_to_pose action server...");
        }

        // Start input thread
        input_thread_ = std::thread(&TurtlebotNavNode::inputListener, this);
    }

    ~TurtlebotNavNode() {
        running_ = false;
        cv_.notify_all();  // Notify all waiting threads to exit
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::thread input_thread_;
    std::atomic<bool> running_;
    bool goal_active_;
    std::mutex goal_mutex_;  // Mutex to protect shared state
    std::condition_variable cv_;  // Condition variable to signal when a goal is set

    void send_goal(float x, float y, float theta) {
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->get_clock()->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.z = sin(theta / 2.0);
        goal_msg.pose.pose.orientation.w = cos(theta / 2.0);

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this, x, y, theta](auto goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected.");
            } else {
                RCLCPP_INFO(this->get_logger(), "New Goal: x=%.2f, y=%.2f, theta=%.2f", x, y, theta);
                {
                    std::lock_guard<std::mutex> lock(goal_mutex_);
                    goal_active_ = true;
                }
                cv_.notify_all();  // Notify odometry callback that a goal is set
            }
        };

        send_goal_options.result_callback = [this](const auto& result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
                {
                    std::lock_guard<std::mutex> lock(goal_mutex_);
                    goal_active_ = false;
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Goal failed with status: %d", static_cast<int>(result.code));
            }
        };

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Wait until a goal is set
        std::unique_lock<std::mutex> lock(goal_mutex_);
        cv_.wait(lock, [this]() { return goal_active_ || !running_; });

        if (!running_) return;  // Exit if the node is shutting down

        // Process odometry data if a goal is active
        if (goal_active_) {
            geometry_msgs::msg::PoseStamped odom_pose;
            odom_pose.header = msg->header;
            odom_pose.pose = msg->pose.pose;

            try {
                geometry_msgs::msg::PoseStamped map_pose;
                map_pose = tf_buffer_.transform(odom_pose, "map", tf2::durationFromSec(1.0));

                double x = map_pose.pose.position.x;
                double y = map_pose.pose.position.y;
                RCLCPP_INFO(this->get_logger(), "Current Position: x=%.2f, y=%.2f", x, y);
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Transform warning: %s", ex.what());
            }
        }
    }

    void inputListener() {
        while (running_) {
            std::string input;
            std::cout << "Enter new goal (format: x y theta): ";
            std::getline(std::cin, input);

            if (!running_) break;

            std::istringstream iss(input);
            float x, y, theta;
            if (iss >> x >> y >> theta) {
                send_goal(x, y, theta);
            } else {
                std::cout << "Invalid input! Use format: x y theta\n";
            }
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtlebotNavNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
