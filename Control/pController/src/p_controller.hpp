#ifndef PCONTROLLER_HPP
#define PCONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>
#include <algorithm>
#include "obstacle_avoidance.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <chrono>
#include <thread>
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"


/**
 * @class pController
 * @brief A class for controlling the TurtleBot using a P-controller and dynamic obstacle avoidance.
 */
class pController : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the P controller node.
     */
    pController(ObstacleAvoidance& obstacleavoidance);

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom);
    void path_callback(const nav_msgs::msg::Path::SharedPtr path);
    void move();

    void mode_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void interrupt_callback(const std_msgs::msg::Bool::SharedPtr msg);

    double getDistanceError(const geometry_msgs::msg::PoseStamped &goal);
    double getAngularError(const geometry_msgs::msg::PoseStamped &goal);
    nav_msgs::msg::Odometry getOdometry(void);

    void dancey_dance();
    void turn_and_look_for_art();

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_table_marker;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr interrupt_sub_;

    double Kp_linear;
    double Kp_angular;
    double tolerance;

    nav_msgs::msg::Odometry odo_;
    nav_msgs::msg::Path path_;
    size_t path_index_ = 0;
    int goals_in_obstacles_ = 0;

    ObstacleAvoidance& obstacleavoidance_;
    visualization_msgs::msg::MarkerArray markerArray_;

    bool obstruction_handled = false;
    bool going_to_safe_goal = false;
    bool new_goal_targeted_ = false;
    bool in_emergency_mode_ = false;
    bool reversing_from_obstacle_ = false;

    geometry_msgs::msg::PoseStamped pending_goal_;

    int mode_ = 0; // 0: default, 1: sentry, 2: dance
    bool interrupted_ = false;
    int received_int_;  // optional: store the last received value
    bool sentry_mode_ = false;

    //rotating 360 degrees member variables
    rclcpp::TimerBase::SharedPtr rotate_timer_;
    bool rotating_ = false;
    double yaw_start_;
    bool first_rotation_step_ = true;
    double yaw_accumulated_ = 0.0;
    double yaw_last_ = 0.0;


    //dancing member variables
    bool dancing_ = false;
    int dance_stage_ = 0;
    int dance_cycles_ = 0;
    double yaw_dance_last_ = 0.0;
    double yaw_dance_accumulated_ = 0.0;
    rclcpp::TimerBase::SharedPtr dance_timer_;
    bool first_dance_step_ = true;


    bool first_goal_ = true;



};

#endif // PCONTROLLER_HPP
