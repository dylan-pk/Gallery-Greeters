#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class AutoLocalise : public rclcpp::Node
{
public:
    AutoLocalise() : Node("auto_localise")
    {
        // Declare and retrieve parameters
        this->declare_parameter<std::string>("robot_namespace", "tb1");
        this->declare_parameter<double>("x", 0.0);
        this->declare_parameter<double>("y", 0.0);
        this->declare_parameter<double>("yaw_deg", 0.0);

        std::string robot_ns;
        this->get_parameter("robot_namespace", robot_ns);
        this->get_parameter("x", x_);
        this->get_parameter("y", y_);
        this->get_parameter("yaw_deg", yaw_deg_);

        std::string topic = "/" + robot_ns + "/initialpose";
        pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(topic, 10);
        RCLCPP_INFO(this->get_logger(), "Initial pose will be published to: %s", topic.c_str());

        // Schedule publishing after 1 second
        timer_ = this->create_wall_timer(1s, std::bind(&AutoLocalise::publish_initial_pose, this));
    }

private:
    void publish_initial_pose()
    {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";

        msg.pose.pose.position.x = x_;
        msg.pose.pose.position.y = y_;
        msg.pose.pose.position.z = 0.0;

        double yaw_rad = yaw_deg_ * M_PI / 180.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_rad);
        msg.pose.pose.orientation = tf2::toMsg(q);

        msg.pose.covariance[0] = 0.25;
        msg.pose.covariance[7] = 0.25;
        msg.pose.covariance[35] = (M_PI / 12.0) * (M_PI / 12.0);  // 15 deg variance

        pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "✅ Published initial pose: (%.2f, %.2f, %.2f°)", x_, y_, yaw_deg_);

        timer_->cancel();  // Only publish once
    }

    double x_, y_, yaw_deg_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoLocalise>());
    rclcpp::shutdown();
    return 0;
}
