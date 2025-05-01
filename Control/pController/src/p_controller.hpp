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
        pController(ObstacleAvoidance& obstacleavoidance);

    private:
    
    /**
     * @brief Callback function to process the Odometry data.
     * 
     * @param odom The incoming odom data.
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom);

    void path_callback(const nav_msgs::msg::Path::SharedPtr path);

    void move();

    visualization_msgs::msg::Marker produceMarkerTable(geometry_msgs::msg::Point pt);
    void publishFinalGoalMarkers();
    geometry_msgs::msg::Point createPoint(double x, double y, double z);
    unsigned int ct_; 

    // double getDistanceError();

    // double getAngularError();

    double getDistanceError(const geometry_msgs::msg::PoseStamped &goal);
    double getAngularError(const geometry_msgs::msg::PoseStamped &goal);

    nav_msgs::msg::Odometry getOdometry(void);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; ///< Subscription to laser scan data.
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_; ///< Subscription to planned path.
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_; ///< Publisher for command velocity.

    double Kp_linear;
    double Kp_angular;
    double target_x;
    double target_y;
    double tolerance;
    nav_msgs::msg::Odometry odo_;
    nav_msgs::msg::Path path_;
    size_t path_index_ = 0;

    int goals_in_obstacles_ = 0;
    rclcpp::TimerBase::SharedPtr timer_;

    ObstacleAvoidance& obstacleavoidance_;

    visualization_msgs::msg::MarkerArray markerArray_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_table_marker;

    bool obstruction_handled = false;
    bool going_to_safe_goal = false;

    bool escaping_collision_ = false;
int escape_direction_ = 0;

double getAngleToGoal(const geometry_msgs::msg::Point &goal);

geometry_msgs::msg::PoseStamped pending_goal_;
bool new_goal_targeted_ = false;


};  

#endif // PCONTROLLER_HPP