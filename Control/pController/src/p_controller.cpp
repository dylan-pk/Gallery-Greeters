#include "p_controller.hpp"

pController::pController(ObstacleAvoidance &obstacleavoidance) : Node("p_controller"), Kp_linear(0.8), Kp_angular(1.0), tolerance(0.1), obstacleavoidance_(obstacleavoidance)
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

    pub_table_marker = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", 1000);
}

// void pController::move()
// {

//     if (path_.poses.empty())
//     {
//         // RCLCPP_WARN(this->get_logger(), "No path received yet.");
//         return;
//     }

//     if (path_index_ >= path_.poses.size())
//     {
//         RCLCPP_INFO(this->get_logger(), "All waypoints reached.");
//         RCLCPP_INFO(this->get_logger(), "goals in obstacles: %d", goals_in_obstacles_);
//         geometry_msgs::msg::Twist stop_msg;
//         cmd_vel_pub_->publish(stop_msg);
//         path_index_ = 0;
//         path_.poses.clear();
//         // publishFinalGoalMarkers();
//         return;
//     }

//     // const auto &goal = path_.poses.at(path_index_);
//     double dist = getDistanceError(path_.poses.at(path_index_));
//     double angle = getAngularError(path_.poses.at(path_index_));

//     RCLCPP_INFO(this->get_logger(), "Goal %zu | Distance: %.2f", path_index_ + 1, dist);

//     if(dist < 0.4){
//     if (obstacleavoidance_.isGoalInsideObstacle(getOdometry(), path_.poses.at(path_index_).pose.position))
//     {
//         RCLCPP_WARN(this->get_logger(), "Current goal %zu is inside an obstacle. Replacing...", path_index_ + 1);
//         goals_in_obstacles_ += 1;
//         geometry_msgs::msg::PoseStamped next_goal;
//         if (path_index_ + 1 < path_.poses.size())
//             next_goal = path_.poses[path_index_ + 1];
//         else
//             next_goal = path_.poses[path_index_]; // fallback to current if no next

//         geometry_msgs::msg::PoseStamped suggested = obstacleavoidance_.suggestNewGoal(path_.poses[path_index_], next_goal);

//         // Replace just the current goal
//         path_.poses.erase(path_.poses.begin() + path_index_);
//         path_.poses.insert(path_.poses.begin() + path_index_, suggested);

//     }
//     }

//     if (dist < tolerance)
//     {
//         RCLCPP_INFO(this->get_logger(), "Reached goal %zu", path_index_);
//         path_index_++;

//         geometry_msgs::msg::Twist stop_msg;
//         cmd_vel_pub_->publish(stop_msg);
//         return;
//     }
//     else
//     {
//         geometry_msgs::msg::Twist cmd_msg;
//         cmd_msg.linear.x = Kp_linear * dist;
//         cmd_msg.angular.z = Kp_angular * angle;
//         cmd_vel_pub_->publish(cmd_msg);

//         if (obstacleavoidance_.isGoalObstructed(getOdometry(), path_.poses.at(path_index_).pose.position))
//         {
//             geometry_msgs::msg::Twist stop_msg;
//             cmd_vel_pub_->publish(stop_msg);

//             geometry_msgs::msg::PoseStamped next_goal;

//             if (path_index_ + 1 < path_.poses.size())
//             {
//                 next_goal = path_.poses.at(path_index_ + 1);
//             }
//             else
//             {
//                 next_goal = path_.poses.at(path_index_); // fallback to current goal
//             }

//             geometry_msgs::msg::PoseStamped temporary_goal = obstacleavoidance_.suggestNewGoal(
//                 path_.poses.at(path_index_),
//                 next_goal);

//             double dist_temp = getDistanceError(temporary_goal);
//             double angle_temp = getAngularError(temporary_goal);

//             if (dist_temp < tolerance)
//             {
//                 geometry_msgs::msg::Twist stop_msg;
//                 cmd_vel_pub_->publish(stop_msg);
//             }

//             geometry_msgs::msg::Twist cmd_msg_temp;
//             cmd_msg_temp.linear.x = Kp_linear * dist_temp;
//             cmd_msg_temp.angular.z = Kp_angular * angle_temp;
//             cmd_vel_pub_->publish(cmd_msg_temp);
//         }
//     }
// }

void pController::move()
{
    if (path_.poses.empty())
        return;

    if (path_index_ >= path_.poses.size())
    {
        RCLCPP_INFO(this->get_logger(), "All waypoints reached.");
        RCLCPP_INFO(this->get_logger(), "goals in obstacles: %d", goals_in_obstacles_);
        geometry_msgs::msg::Twist stop_msg;
        cmd_vel_pub_->publish(stop_msg);
        path_index_ = 0;
        path_.poses.clear();
        return;
    }

    double dist = getDistanceError(path_.poses.at(path_index_));
    double angle = getAngularError(path_.poses.at(path_index_));

    // RCLCPP_INFO(this->get_logger(), "Goal %zu | Distance: %.2f", path_index_ + 1, dist);

    if (dist < 0.5)
    {
        if (obstacleavoidance_.isGoalInsideObstacle(getOdometry(), path_.poses.at(path_index_).pose.position))
        {
            RCLCPP_WARN(this->get_logger(), "Current goal %zu is inside an obstacle. Replacing...", path_index_ + 1);
            goals_in_obstacles_++;

            geometry_msgs::msg::PoseStamped next_goal;
            if (path_index_ + 1 < path_.poses.size())
                next_goal = path_.poses[path_index_ + 1];
            else
                next_goal = path_.poses[path_index_];

            geometry_msgs::msg::PoseStamped suggested = obstacleavoidance_.suggestNewGoal(
                path_.poses[path_index_], next_goal);

            // Replace just the current goal
            path_.poses[path_index_] = suggested;
        }
    }

    if (dist < tolerance)
    {
        RCLCPP_INFO(this->get_logger(), "Reached goal %zu", path_index_ + 1);
        path_index_++;
        geometry_msgs::msg::Twist stop_msg;
        cmd_vel_pub_->publish(stop_msg);
        return;
    }

    geometry_msgs::msg::Twist cmd_msg;
    cmd_msg.linear.x = Kp_linear * dist;
    cmd_msg.angular.z = Kp_angular * angle;

    if (cmd_msg.linear.x > 0.08)
    {
        cmd_msg.linear.x = 0.08;
    }

    if (cmd_msg.angular.z > 0.3)
    {
        cmd_msg.angular.z = 0.3;
    }

        if (cmd_msg.linear.x < 0.03)
    {
        cmd_msg.linear.x = 0.03;
    }

    // Smooth velocity command (reduce jerkiness)
    static geometry_msgs::msg::Twist prev_cmd;
    const double SMOOTHING_ALPHA = 0.7;
    cmd_msg.linear.x = SMOOTHING_ALPHA * cmd_msg.linear.x + (1 - SMOOTHING_ALPHA) * prev_cmd.linear.x;
    cmd_msg.angular.z = SMOOTHING_ALPHA * cmd_msg.angular.z + (1 - SMOOTHING_ALPHA) * prev_cmd.angular.z;
    prev_cmd = cmd_msg;

    cmd_vel_pub_->publish(cmd_msg);

    // --- Re-evaluate if obstacle has appeared in front of the goal ---
    if (obstacleavoidance_.isGoalObstructed(getOdometry(), path_.poses.at(path_index_).pose.position))
    {
        geometry_msgs::msg::Twist stop_msg;
        cmd_vel_pub_->publish(stop_msg);

        geometry_msgs::msg::PoseStamped next_goal;
        if (path_index_ + 1 < path_.poses.size())
            next_goal = path_.poses.at(path_index_ + 1);
        else
            next_goal = path_.poses.at(path_index_);

        geometry_msgs::msg::PoseStamped temporary_goal = obstacleavoidance_.suggestNewGoal(
            path_.poses.at(path_index_), next_goal);

        double dist_temp = getDistanceError(temporary_goal);
        double angle_temp = getAngularError(temporary_goal);

        geometry_msgs::msg::Twist cmd_msg_temp;
        cmd_msg_temp.linear.x = Kp_linear * dist_temp;
        cmd_msg_temp.angular.z = Kp_angular * angle_temp;

        if (cmd_msg_temp.linear.x > 0.08)
        {
            cmd_msg_temp.linear.x = 0.08;
        }

        if (cmd_msg_temp.angular.z > 0.3)
        {
            cmd_msg_temp.angular.z = 0.3;
        }

        // Smooth temp command
        cmd_msg_temp.linear.x = SMOOTHING_ALPHA * cmd_msg_temp.linear.x + (1 - SMOOTHING_ALPHA) * prev_cmd.linear.x;
        cmd_msg_temp.angular.z = SMOOTHING_ALPHA * cmd_msg_temp.angular.z + (1 - SMOOTHING_ALPHA) * prev_cmd.angular.z;
        prev_cmd = cmd_msg_temp;

        cmd_vel_pub_->publish(cmd_msg_temp);
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
    while (angular_error > M_PI)
        angular_error -= 2 * M_PI;
    while (angular_error < -M_PI)
        angular_error += 2 * M_PI;

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

visualization_msgs::msg::Marker pController::produceMarkerTable(geometry_msgs::msg::Point pt)
{

    visualization_msgs::msg::Marker marker;

    // We need to set the frame
    //  Set the frame ID and time stamp.
    marker.header.frame_id = "odom";
    marker.header.stamp = this->get_clock()->now();
    // We set lifetime (it will dissapear in this many seconds)
    marker.lifetime = rclcpp::Duration(1000, 0); // zero is forever

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "goals"; // This is namespace, markers can be in diofferent namespace
    marker.id = ct_++;   // We need to keep incrementing markers to send others ... so THINK, where do you store a vaiable if you need to keep incrementing it

    // The marker type
    marker.type = visualization_msgs::msg::Marker::CYLINDER;

    // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = pt.x;
    marker.pose.position.y = pt.y;
    marker.pose.position.z = pt.z;

    // Orientation, we are not going to orientate it, for a quaternion it needs 0,0,0,1
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1m side
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 2.0;

    // Let's send a marker with color (green for reachable, red for now)
    std_msgs::msg::ColorRGBA color;
    color.a = 0.5; // a is alpha - transparency 0.5 is 50%;
    color.r = 250.0 / 255.0;
    color.g = 0;
    color.b = 0;

    marker.color = color;

    return marker;
}

void pController::publishFinalGoalMarkers()
{
    std::vector<geometry_msgs::msg::Point> marker_points = {
        createPoint(0.81, 1.21, 0.0),
        createPoint(0.9, 2.43, 0.0),
        createPoint(2.0, 1.8, 0.0),
        createPoint(2.91, 1.0, 0.0),
        createPoint(2.91, 2.2, 0.0)

        // tables 1-5
    };

    std::vector<geometry_msgs::msg::Point> robot_positions_at_table = {
        createPoint(1.22, 1.22, 0.0),
        createPoint(1.25, 2.18, 0.0),
        createPoint(1.7, 1.6, 0.0),
        createPoint(2.66, 1.34, 0.0),
        createPoint(2.76, 1.8, 0.0)

        // tables 1-5
    };

    visualization_msgs::msg::MarkerArray marker_array;
    for (const auto &pt : marker_points)
    {
        marker_array.markers.push_back(produceMarkerTable(pt));
    }

    pub_table_marker->publish(marker_array);
}

geometry_msgs::msg::Point pController::createPoint(double x, double y, double z)
{
    geometry_msgs::msg::Point pt;
    pt.x = x;
    pt.y = y;
    pt.z = z;
    return pt;
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
