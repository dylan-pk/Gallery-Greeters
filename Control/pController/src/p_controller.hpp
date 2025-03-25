#ifndef PCONTROLLER_HPP
#define PCONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

/**
 * @class P Controller
 * @brief A class for controlling the turtlebot through the use of a P-controller
 */

class pController : public rclcpp::Node
{
    public:
        /**
         * @brief Constructor for the P controller node
         */
        pController(double x_target, double y_target);

    private:
    
    /**
     * @brief Callback function to process the Odometry data.
     * 
     * @param odom The incoming odom data.
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; ///< Subscription to laser scan data.
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_; ///< Publisher for command velocity.

    double Kp_linear;
    double Kp_angular;
    double target_x;
    double target_y;
    double tolerance;
};

#endif // PCONTROLLER_HPP