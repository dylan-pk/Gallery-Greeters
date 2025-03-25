#ifndef OBSTACLE_DETECTION_H
#define OBSTACLE_DETECTION_H

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>

class ObstacleDetection : public rclcpp::Node{

    public:

    ObstacleDetection();



    private:

    void laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;//!< Pointer to the laser scan subscriber


};

#endif 
