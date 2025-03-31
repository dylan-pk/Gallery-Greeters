#ifndef OBSTACLE_DETECTION_H
#define OBSTACLE_DETECTION_H

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/utils.h> //To use getYaw function from the quaternion of orientation
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "visualization_msgs/msg/marker_array.hpp"

class ObstacleDetection : public rclcpp::Node{

    public:

    ObstacleDetection();



    private:

    void laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg);

    void odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);

    nav_msgs::msg::Odometry getOdometry(void);
    
    geometry_msgs::msg::Point localToGlobal(nav_msgs::msg::Odometry global, geometry_msgs::msg::Point local);

    unsigned int countSegments(nav_msgs::msg::Odometry odom);

    visualization_msgs::msg::Marker produceMarker(geometry_msgs::msg::Point pt);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;//!< Pointer to the laser scan subscriber

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odo_;

    sensor_msgs::msg::LaserScan scan_;

    std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> segmentVector_;

    unsigned int ct_; //!< Marker Count

    visualization_msgs::msg::MarkerArray markerArray_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker;

    nav_msgs::msg::Odometry odo_;



};

#endif 
