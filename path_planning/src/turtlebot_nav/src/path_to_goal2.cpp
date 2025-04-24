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
// #include "tf2 LinearMath/Quaternion.h"
// #include "tf2 utils.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

using namespace std::chrono_literals;

class PathToGoalClient : public rclcpp::Node {
public:
    PathToGoalClient()
        : Node("path_to_goal_client"), goal_active_(false),
          tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_), running_(true) {
        
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(this, "/compute_path_to_pose");

        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&PathToGoalClient::odometry_callback, this, std::placeholders::_1));
        
        goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pose_topic", 10, std::bind(&PathToGoalClient::pose_callback, this, std::placeholders::_1));

        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("computed_path", 10);
    }
    void start() {
        // Wait for action server
        while (!action_client_->wait_for_action_server(2s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for the compute_path_to_pose action server...");
        }

        // Wait for odometry
        RCLCPP_INFO(this->get_logger(), "Waiting for odometry...");
        rclcpp::Rate rate(10);
        for (int i = 0; i < 100; ++i) {
            {
                std::lock_guard<std::mutex> lock(odom_mutex_);
                if (latest_odom_) {
                    RCLCPP_INFO(this->get_logger(), "Odometry received.");
                    break;
                }
            }
            rate.sleep();
        }

        rclcpp::sleep_for(1s);  // Give some buffer time
        publish_initial_pose();
    }

    ~PathToGoalClient() {
        running_ = false;
        // cv_.notify_all();
    }

private:
    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr action_client_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
    nav_msgs::msg::Path computed_path;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::atomic<bool> running_;
    bool goal_active_;
    std::mutex goal_mutex_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
    std::mutex odom_mutex_;
    
    // std::condition_variable cv_;

    void publish_initial_pose() {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (!latest_odom_) {
            RCLCPP_WARN(this->get_logger(), "No odometry data received yet. Skipping initial pose publish.");
            return;
        }
    
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";  // Make sure your TF tree supports map->odom transform
    
        // Use odometry position
        msg.pose.pose.position = latest_odom_->pose.pose.position;
        msg.pose.pose.orientation = latest_odom_->pose.pose.orientation;
    
        // Use a reasonable default covariance
        msg.pose.covariance = {
            0.25, 0, 0, 0, 0, 0,
            0, 0.25, 0, 0, 0, 0,
            0, 0, 0.0, 0, 0, 0,
            0, 0, 0, 0.0685, 0, 0,
            0, 0, 0, 0, 0.0685, 0,
            0, 0, 0, 0, 0, 0.0685
        };
    
        initial_pose_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published initial pose from odometry.");
        
        // auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        // msg.header.stamp = this->now();
        // msg.header.frame_id = "map";
        // msg.pose.pose.position.x = -2.0;
        // msg.pose.pose.position.y = -0.5;
        // msg.pose.pose.position.z = 0.0;
        // msg.pose.pose.orientation.z = 0.0;
        // msg.pose.pose.orientation.w = 1.0;
        // msg.pose.covariance = {
        //     0.25, 0, 0, 0, 0, 0,
        //     0, 0.25, 0, 0, 0, 0,
        //     0, 0, 0.0, 0, 0, 0,
        //     0, 0, 0, 0.0685, 0, 0,
        //     0, 0, 0, 0, 0.0685, 0,
        //     0, 0, 0, 0, 0, 0.0685
        // };
        // initial_pose_pub_->publish(msg);
        // RCLCPP_INFO(this->get_logger(), "Published initial pose.");
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // if (goal_active_) {
        //     RCLCPP_INFO(this->get_logger(), "Goal already active, ignoring new goal.");
        //     return;
        // }
    
        goal_active_ = true;
        RCLCPP_INFO(this->get_logger(), "Received goal pose, sending to Nav2.");
    
        auto goal_msg = nav2_msgs::action::ComputePathToPose::Goal();
        goal_msg.goal = *msg;
        goal_msg.goal.header.stamp = this->now();
        goal_msg.goal.header.frame_id = "map";
    
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this](auto result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Path successfully computed, publishing...");
                this->computed_path = result.result->path;
                this->path_publisher_->publish(this->computed_path);
                RCLCPP_INFO(this->get_logger(), "Published path.");
                // rclcpp::shutdown();  //  Shutdown right after publishing path
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to compute path.");
                goal_active_ = false;
            }
        };
    
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }
    
        

    void send_goal(float x, float y, float theta) {
        if (!action_client_->wait_for_action_server(2s)) {
            RCLCPP_WARN(this->get_logger(), "Action server not available.");
            return;
        }
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
                // cv_.notify_all();
            };

        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            goal_active_ = true;
        }

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    // void inputListener() {
    //     std::string input;
    //     while (running_) {
    //         std::cout << "Enter goal (x y theta in radians): ";
    //         std::getline(std::cin, input);
    //         std::istringstream iss(input);
    //         float x, y, theta;
    //         if (iss >> x >> y >> theta) {
    //             send_goal(x, y, theta);
    //         } else {
    //             std::cout << "Invalid input. Please enter: x y theta\n";
    //         }
    //     }
    // }

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        latest_odom_ = msg;
        // Optional debug
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received odometry: (%.2f, %.2f)",
                             msg->pose.pose.position.x, msg->pose.pose.position.y);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathToGoalClient>();

    // Start spin thread
    std::thread spin_thread([&]() {
        rclcpp::spin(node);
    });

    // Start logic after spin is running
    node->start();

    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
