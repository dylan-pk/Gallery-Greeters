#include "p_controller.hpp"

    pController::pController(double x_target, double y_target) : Node("p_controller"), Kp_linear(0.2), Kp_angular(0.2), tolerance(0.1), target_x(x_target), target_y(y_target)
    {  
        
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Create a subscriber to get odometry data
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&pController::odom_callback, this, std::placeholders::_1));


    }

    void pController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {

        double current_x = odom->pose.pose.position.x;
        double current_y = odom->pose.pose.position.y;

        double error_x = target_x - current_x;
        double error_y = target_y - current_y;
        double distance_error = std::sqrt(error_x * error_x + error_y * error_y);
        
        RCLCPP_INFO(this->get_logger(), "current pos is  (%.2f, %.2f)", current_x, current_y);
        RCLCPP_INFO(this->get_logger(), "goal is  (%.2f, %.2f)", target_x, target_y);
        RCLCPP_INFO(this->get_logger(), "Distance to goal is  %.2f", distance_error);

        // double Kp = 0.1;
        // double tolerance = 0.05;

        if (distance_error < tolerance)
        {
            RCLCPP_INFO(this->get_logger(), "Target reached.");
            geometry_msgs::msg::Twist stop_msg;
            cmd_vel_pub_->publish(stop_msg);
            return;
        }

        double desired_theta = std::atan2(error_y, error_x);

        // Get current robot orientation (convert quaternion to yaw)
        tf2::Quaternion q(
            odom->pose.pose.orientation.x,
            odom->pose.pose.orientation.y,
            odom->pose.pose.orientation.z,
            odom->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, current_theta;
        m.getRPY(roll, pitch, current_theta);

        // Compute angular error
        double angular_error = desired_theta - current_theta;
        while (angular_error > M_PI) angular_error -= 2 * M_PI;
        while (angular_error < -M_PI) angular_error += 2 * M_PI;

        geometry_msgs::msg::Twist cmd_msg;
        cmd_msg.linear.x = Kp_linear * distance_error;
        cmd_msg.angular.z = Kp_angular * angular_error;

        // Publish velocity command
        cmd_vel_pub_->publish(cmd_msg);





    }

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    double target_x = std::stod(argv[1]);
    double target_y = std::stod(argv[2]);
    rclcpp::spin(std::make_shared<pController>(target_x, target_y));
    rclcpp::shutdown();
    return 0;
}

//consider using moveIT