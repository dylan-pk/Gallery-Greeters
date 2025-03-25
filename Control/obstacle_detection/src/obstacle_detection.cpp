#include "obstacle_detection.hpp"
    
ObstacleDetection::ObstacleDetection() : Node("obstacle_detection"){

    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/odom", 10, std::bind(&ObstacleDetection::laserCallback, this, std::placeholders::_1));


}

void ObstacleDetection::laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg)
{
    // std::unique_lock<std::mutex> lck(laserData_.mtx);
    // laserData_.scan = *msg;
    // sensor_msgs::msg::LaserScan scan = laserData_.scan;
    // lck.unlock();

    // if(laserProcessingPtr_ == nullptr){
    //     laserProcessingPtr_ = std::make_unique<LaserProcessing>(scan);
    // } else {
    //     laserProcessingPtr_->newScan(scan);
    // }

    sensor_msgs::msg::LaserScan scan = *msg;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleDetection>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}