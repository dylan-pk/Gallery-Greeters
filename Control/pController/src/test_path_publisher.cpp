#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <vector>

class TestPathPublisher : public rclcpp::Node
{
public:
  TestPathPublisher() : Node("test_path_publisher")
  {
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("computed_path", 10);
    publish_test_path();
  }

  void publish_test_path()
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";

    std::vector<std::tuple<double, double, double>> waypoints = {
        // {1.0, 0.5, 0.0},
        // {3.0, 1.0, M_PI_4},
        // {6.0, 1.5, M_PI_2},
        // {4.0, 1.0, M_PI},
        // {5.0, 0.5, -M_PI_2}

        // testing in turtlebot world
        // {-0.5, 0.0, 0.0},
        // {0.5, 0.0, 0.0},
        // {1.5, 0.0, 0.0},

        // random spot in the middle of gallery
        // {1.5, 2.0, 0.0},

        // positions near tables 1-5 //maybe get orientation?
        // {0.81, 1.21, 0.0},
        // {1.22, 1.22, 0.0},
        // {1.25, 2.18, 0.0},
        // {1.7, 1.6, 0.0},
        // {2.66, 1.34, 0.0},
        // {2.76, 1.8, 0.0}


        //path from origin

        {1, 0.8, 0.0},
        {1.3, 1, 0.0},
        {1.57, 1.16, 0.0},
        {1.77, 1.33, 0.0},
        {2, 1.8, 0.0},
        {2.4, 2.23, 0.0}

        // tables 1-5
    };

    for (const auto &wp : waypoints)
    {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.stamp = this->now();
      pose_stamped.header.frame_id = "map";

      double x, y, yaw;
      std::tie(x, y, yaw) = wp;
      pose_stamped.pose.position.x = x;
      pose_stamped.pose.position.y = y;

      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      pose_stamped.pose.orientation.x = q.x();
      pose_stamped.pose.orientation.y = q.y();
      pose_stamped.pose.orientation.z = q.z();
      pose_stamped.pose.orientation.w = q.w();

      path_msg.poses.push_back(pose_stamped);
    }

    RCLCPP_INFO(this->get_logger(), "Publishing path with %zu waypoints...", path_msg.poses.size());
    path_pub_->publish(path_msg);

    // Sleep briefly to ensure message delivery before shutdown
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TestPathPublisher>();

  // No spin needed — we only publish once and exit
  rclcpp::shutdown();
  return 0;
}