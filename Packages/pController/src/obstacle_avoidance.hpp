// #ifndef OBSTACLE_AVOIDANCE_HPP
// #define OBSTACLE_AVOIDANCE_HPP

// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <geometry_msgs/msg/pose_array.hpp>
// #include <geometry_msgs/msg/point.hpp>
// #include <nav_msgs/msg/occupancy_grid.hpp>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/buffer.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <nav_msgs/msg/odometry.hpp>

// // swithching to service as static grid isnt always sending
// #include "std_srvs/srv/trigger.hpp"

// //adding the flann approach
// #include <opencv2/flann/flann.hpp>
// #include <opencv2/core.hpp>
// #include <opencv2/opencv.hpp>


// class ObstacleAvoidance : public rclcpp::Node
// {
// public:
//     ObstacleAvoidance();

// private:
//     void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
//     void staticObstaclesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
//     void staticMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
//     void publishUnknownObstacles();

//     bool is_known_static_obstacle(const geometry_msgs::msg::Point &pt, double threshold) const;
//     std::vector<geometry_msgs::msg::Point> getFilteredLaserPoints() const;

//     // ROS
//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
//     rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr static_obs_sub_;
//     rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr static_map_sub_;
//     rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr unknown_grid_pub_;
//     rclcpp::TimerBase::SharedPtr timer_;

//     // TF
//     std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
//     std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

//     // Data
//     sensor_msgs::msg::LaserScan latest_scan_;
//     nav_msgs::msg::MapMetaData map_info_;
//     bool map_ready_ = false;
//     std::vector<geometry_msgs::msg::Point> known_static_obstacles_;


//     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
// nav_msgs::msg::Odometry current_odom_;

// void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

// //for reuesting occupancy grid from pathtogoal4 service
// rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr static_grid_client_;
// void requestStaticGridOnce();
// rclcpp::TimerBase::SharedPtr request_timer_;

// cv::Mat known_obstacles_mat_;
// std::unique_ptr<cv::flann::Index> kdtree_;

// std::vector<geometry_msgs::msg::Point> clusterPoints(
//     const std::vector<geometry_msgs::msg::Point> &points, double cluster_radius, int min_cluster_size);

//  cv::Mat static_map_grid_;
//  rclcpp::TimerBase::SharedPtr visual_timer_;
// void displayStaticMap();

// };

// #endif



#ifndef OBSTACLE_AVOIDANCE_HPP
#define OBSTACLE_AVOIDANCE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>
#include "std_srvs/srv/trigger.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>


class ObstacleAvoidance : public rclcpp::Node
{
public:
    ObstacleAvoidance();

private:
    // === Callback functions ===
    void staticMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void localCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    // === Timed routines ===
    void publishUnknownObstacles();
    // void displayStaticMap();

    void displayUnknownMap();

    // === Subscriptions ===
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr static_map_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;

    // === Publisher ===
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr unknown_grid_pub_;

    // === Timers ===
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr visual_timer_;

    // === Internal map storage ===
    nav_msgs::msg::OccupancyGrid static_map_;
    nav_msgs::msg::OccupancyGrid local_costmap_;

    // === State flags ===
    bool static_map_ready_ = false;
    bool local_costmap_ready_ = false;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr static_grid_client_;
rclcpp::TimerBase::SharedPtr request_timer_;
void requestStaticGridOnce();


//for ttl logic (getting rid of old obstacles)
std::vector<rclcpp::Time> last_seen_;

//adding laserscan verificaiton
sensor_msgs::msg::LaserScan latest_scan_;

std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);



};

#endif  // OBSTACLE_AVOIDANCE_HPP
