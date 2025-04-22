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

      {-0.5, 0.0, 0.0},
      {0.5, 0.0, 0.0},
      {1.5, 0.0, 0.0},
      };

    for (const auto & wp : waypoints)
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