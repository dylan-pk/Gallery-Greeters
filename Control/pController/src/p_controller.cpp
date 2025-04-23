#include "p_controller.hpp"

pController::pController(ObstacleAvoidance& obstacleavoidance) : Node("p_controller"), Kp_linear(0.2), Kp_angular(0.4), tolerance(0.1), obstacleavoidance_(obstacleavoidance)
{

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Create a subscriber to get odometry data
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&pController::odom_callback, this, std::placeholders::_1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "computed_path", 10, std::bind(&pController::path_callback, this, std::placeholders::_1));
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&pController::move, this));

    
}

void pController::move()
{
    if (path_.poses.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No path received yet.");
        return;
    }

    if (path_index_ >= path_.poses.size())
    {
        RCLCPP_INFO(this->get_logger(), "All waypoints reached.");
        geometry_msgs::msg::Twist stop_msg;
        cmd_vel_pub_->publish(stop_msg);
        path_index_ = 0;
        path_.poses.clear();
        return;
    }

    // const auto &goal = path_.poses.at(path_index_);    
    double dist = getDistanceError(path_.poses.at(path_index_));
    double angle = getAngularError(path_.poses.at(path_index_));

    RCLCPP_INFO(this->get_logger(), "Goal %zu | Distance: %.2f", path_index_ + 1, dist);

    if (dist < tolerance)
    {
        RCLCPP_INFO(this->get_logger(), "Reached goal %zu", path_index_);
        path_index_++;

        geometry_msgs::msg::Twist stop_msg;
        cmd_vel_pub_->publish(stop_msg);
        return;
    }     else
    {
        geometry_msgs::msg::Twist cmd_msg;
        cmd_msg.linear.x = Kp_linear * dist;
        cmd_msg.angular.z = Kp_angular * angle;
        cmd_vel_pub_->publish(cmd_msg);
        
        if(obstacleavoidance_.isGoalObstructed(getOdometry(), path_.poses.at(path_index_).pose.position)){
            geometry_msgs::msg::Twist stop_msg;
            cmd_vel_pub_->publish(stop_msg);

            // geometry_msgs::msg::PoseStamped temporary_goal = obstacleavoidance_.suggestNewGoal(path_.poses.at(path_index_), path_.poses.at(path_index_ + 1));

            geometry_msgs::msg::PoseStamped next_goal;

            if (path_index_ + 1 < path_.poses.size()) {
                next_goal = path_.poses.at(path_index_ + 1);
            } else {
                next_goal = path_.poses.at(path_index_);  // fallback to current goal
            }

            geometry_msgs::msg::PoseStamped temporary_goal = obstacleavoidance_.suggestNewGoal(
                path_.poses.at(path_index_), 
                next_goal);

            double dist_temp = getDistanceError(temporary_goal);
            double angle_temp = getAngularError(temporary_goal);


            if(dist_temp < tolerance){
                geometry_msgs::msg::Twist stop_msg;
                cmd_vel_pub_->publish(stop_msg);
                
                // if(path_index_ <= path_.poses.size() - 1){
                //     path_index_++;
                //     return;
                // }else{
                //     return;
                // }
            }

            geometry_msgs::msg::Twist cmd_msg_temp;
            cmd_msg_temp.linear.x = Kp_linear * dist_temp;
            cmd_msg_temp.angular.z = Kp_angular * angle_temp;
            cmd_vel_pub_->publish(cmd_msg_temp);

        }

    }

}

double pController::getDistanceError(const geometry_msgs::msg::PoseStamped &goal)
{
        double current_x = getOdometry().pose.pose.position.x;
        double current_y = getOdometry().pose.pose.position.y;

        double error_x = goal.pose.position.x - current_x;
        double error_y = goal.pose.position.y - current_y;
        double distance_error = std::sqrt(error_x * error_x + error_y * error_y);

        return distance_error;
}


double pController::getAngularError(const geometry_msgs::msg::PoseStamped &goal)
{
        double error_x = goal.pose.position.x - getOdometry().pose.pose.position.x;
        double error_y = goal.pose.position.y - getOdometry().pose.pose.position.y;

        double desired_theta = std::atan2(error_y, error_x);

        // Get current robot orientation (convert quaternion to yaw)
        tf2::Quaternion q(
            getOdometry().pose.pose.orientation.x,
            getOdometry().pose.pose.orientation.y,
            getOdometry().pose.pose.orientation.z,
            getOdometry().pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, current_theta;
        m.getRPY(roll, pitch, current_theta);

        // Compute angular error
        double angular_error = desired_theta - current_theta;
        while (angular_error > M_PI) angular_error -= 2 * M_PI;
        while (angular_error < -M_PI) angular_error += 2 * M_PI;

        return angular_error;
    }


void pController::odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
{
    // RCLCPP_INFO(this->get_logger(), "x: %.2f | y: %.2f", msg->pose.pose.position.x, msg->pose.pose.position.y);
    odo_ = *msg;
}

void pController::path_callback(const std::shared_ptr<nav_msgs::msg::Path> path)
{

    path_ = *path;
}

nav_msgs::msg::Odometry pController::getOdometry(void)
{

    nav_msgs::msg::Odometry pose = odo_;
    return pose;
}

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     auto avoidance = std::make_shared<ObstacleAvoidance>();
//     rclcpp::spin(std::make_shared<pController>(*avoidance));
//     rclcpp::shutdown();
//     return 0;
// }

// main.cpp
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    auto avoidance_node = std::make_shared<ObstacleAvoidance>();
    auto controller_node = std::make_shared<pController>(*avoidance_node);
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(avoidance_node);
    executor.add_node(controller_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}


// consider using moveIT
// wrapping the angle constricting to -180 to 180

