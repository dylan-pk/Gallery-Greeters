#ifndef OBSTACLEAVOIDANCE_HPP
#define OBSTACLEAVOIDANCE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2/utils.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>
#include <algorithm>


class ObstacleAvoidance: public rclcpp::Node
{
public:

    ObstacleAvoidance();

    virtual bool isGoalObstructed(const nav_msgs::msg::Odometry &odom, const geometry_msgs::msg::Point &goal);

    bool isGoalInsideObstacle(const nav_msgs::msg::Odometry &odom, const geometry_msgs::msg::Point &goal);
    
    virtual geometry_msgs::msg::Twist adjustVelocity(const geometry_msgs::msg::Twist &original_cmd);
    
    virtual geometry_msgs::msg::PoseStamped suggestNewGoal(const geometry_msgs::msg::PoseStamped &original_goal, const geometry_msgs::msg::PoseStamped &next_goal);

private:

    void laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg);

    void odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);

    nav_msgs::msg::Odometry getOdometry(void);
    
    geometry_msgs::msg::Point localToGlobal(nav_msgs::msg::Odometry global, geometry_msgs::msg::Point local) const;

    std::vector<geometry_msgs::msg::Point> getLaserPoints() const;

    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odo_;
    sensor_msgs::msg::LaserScan scan_;
    nav_msgs::msg::Odometry odo_;
};

#endif // OBSTACLEAVOIDANCE_HPP