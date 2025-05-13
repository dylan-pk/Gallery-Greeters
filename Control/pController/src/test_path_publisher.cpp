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
    // path_pub_ = this->create_publisher<nav_msgs::msg::Path>("computed_path", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("sentry_path", 10);
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

        // {1, 0.8, 0.0},
        // {1.3, 1, 0.0},
        // {1.57, 1.16, 0.0},
        // {1.77, 1.33, 0.0},
        // {2, 1.8, 0.0},
        // {2.4, 2.23, 0.0}

        
        
        
        
        
        // {2.4, 2.23, 0.0},
        // // {2.23, 2, 0.0},
        // {1.9, 1.55, 0.0}, //inside table
        // {1.4, 1.16, 0.0},
        // {1.3, 1, 0.0},
        // {1, 0.8, 0.0}




    //testing goals
    // {2.4, 2.23, 0.0},
    // {2.35, 2.18, 0.0},
    // {2.3, 2.13, 0.0},
    // {2.25, 2.08, 0.0},
    // {2.2, 2.03, 0.0},
    // {2.1, 1.95, 0.0},
    // {2.0, 1.85, 0.0},
    // {1.95, 1.8, 0.0},
    // {1.9, 1.75, 0.0},
    // {1.87, 1.7, 0.0},
    // {1.84, 1.65, 0.0},
    // {1.81, 1.6, 0.0},
    // {1.78, 1.55, 0.0},
    // {1.75, 1.5, 0.0},
    // {1.7, 1.45, 0.0},
    // {1.65, 1.4, 0.0},
    // {1.6, 1.35, 0.0},
    // {1.55, 1.3, 0.0},
    // {1.5, 1.25, 0.0},
    // {1.45, 1.2, 0.0},
    // {1.4, 1.16, 0.0},  // Given point
    // {1.38, 1.13, 0.0},
    // {1.36, 1.1, 0.0},
    // {1.34, 1.07, 0.0},
    // {1.32, 1.04, 0.0},
    // {1.3, 1.0, 0.0},   // Given point
    // {1.26, 0.98, 0.0},
    // {1.22, 0.96, 0.0},
    // {1.18, 0.94, 0.0},
    // {1.14, 0.92, 0.0},
    // {1.1, 0.9, 0.0},
    // {1.05, 0.85, 0.0},
    // {1.0, 0.8, 0.0} 


        // {3.1, 3.25, 0.0} //charging station



      //sentry mode goals
      {3.34, 2.45, 0.0},
      {3.36, 1.76, 0.0},
      {3.38, 1.75, 0.0},
      {3.0, 0.88, 0.0},
      {1.78, 0.93, 0.0},
      {0.6, 1.05, 0.0},
      // {0.5, 1.84, 0.0},
      {0.48, 2.61, 0.0},
      {2.0, 2.39, 0.0},
      {3.34, 2.45, 0.0}




// 
          // position:
      // x: 0.5360413134142046
      // y: 0.5577182565547301
      // z: 0.11787779237489605
    // orientation:
      // x: 0.0028530754959622245
      // y: 0.0013306447077517357
      // z: -0.906416253060795
      // w: 0.4223738457008259
  // covariance:
// 





        //ros2 topic pub /pose_topic geometry_msgs/msg/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 2.4, y: 2.23, z: 0.0}, orientation: {w: 1.0}}}' --once //lachy's code testing

        //ros2 topic pub /robot_mode std_msgs/msg/Int32 "{data: 1}" --once //testing sentry/dance mode
        //ros2 topic pub /interrupt_signal std_msgs/msg/Bool "{data: True}" --once




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