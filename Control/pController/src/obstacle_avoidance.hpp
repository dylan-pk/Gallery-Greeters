#ifndef OBSTACLE_AVOIDANCE_HPP
#define OBSTACLE_AVOIDANCE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>

class ObstacleAvoidance : public rclcpp::Node
{
public:
    ObstacleAvoidance();

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void staticObstaclesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void staticMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void publishUnknownObstacles();

    bool is_known_static_obstacle(const geometry_msgs::msg::Point &pt, double threshold) const;
    std::vector<geometry_msgs::msg::Point> getFilteredLaserPoints() const;

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr static_obs_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr static_map_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr unknown_grid_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Data
    sensor_msgs::msg::LaserScan latest_scan_;
    nav_msgs::msg::MapMetaData map_info_;
    bool map_ready_ = false;
    std::vector<geometry_msgs::msg::Point> known_static_obstacles_;


    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
nav_msgs::msg::Odometry current_odom_;

void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

};

#endif
