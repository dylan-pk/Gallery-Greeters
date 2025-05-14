#include "p_controller.hpp"

pController::pController(ObstacleAvoidance &obstacleavoidance) : Node("p_controller"), Kp_linear(1.25), Kp_angular(1.0), tolerance(0.1), obstacleavoidance_(obstacleavoidance)
{

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Create a subscriber to get odometry data
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&pController::odom_callback, this, std::placeholders::_1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "computed_path", 10, std::bind(&pController::path_callback, this, std::placeholders::_1));

    mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/robot_mode", 10, std::bind(&pController::mode_callback, this, std::placeholders::_1));

    interrupt_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/interrupt_signal", 10, std::bind(&pController::interrupt_callback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pose_topic", 10, std::bind(&pController::goal_callback, this, std::placeholders::_1));

    reached_artwork_pub_ = this->create_publisher<std_msgs::msg::Bool>("/reaches", 10);

    client_ = this->create_client<std_srvs::srv::SetBool>("/perform_template_matching");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&pController::move, this));

    pub_table_marker = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", 1000);
}

void pController::interrupt_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    interrupted_ = msg->data;
}

void pController::goal_callback(const geometry_msgs::msg::PoseStamped msg)
{
    final_goal_orientation_ = msg.pose.orientation;
    has_final_orientation_ = true;
}

void pController::mode_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    received_int_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received control mode: %d", received_int_);

    if (received_int_ == 0)
    {
        // sentry_mode_ = true;
        going_to_artwork_ = true;
        RCLCPP_INFO(this->get_logger(), "Going to Artwork");
    }

    else if (received_int_ == 1)
    {
        sentry_mode_ = true;
        interrupted_ = false;
        RCLCPP_INFO(this->get_logger(), "Sentry Mode Activated");
    }

    else if (received_int_ == 2)
    {
        sentry_mode_ = false;
        RCLCPP_INFO(this->get_logger(), "💃 Dance mode activated!");

        dancing_ = true;
        dance_cycles_ = 0;
        dance_stage_ = 0; // 0 = rotate left, 1 = rotate right
        yaw_dance_accumulated_ = 0.0;
        first_dance_step_ = true;

        dance_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&pController::dancey_dance, this));
    }

    else if (received_int_ == 3)
    {
        sentry_mode_ = false;
        RCLCPP_INFO(this->get_logger(), "Starting 360° rotation...");

        tf2::Quaternion q_start(
            odo_.pose.pose.orientation.x,
            odo_.pose.pose.orientation.y,
            odo_.pose.pose.orientation.z,
            odo_.pose.pose.orientation.w);
        tf2::Matrix3x3 m_start(q_start);
        double roll, pitch;
        m_start.getRPY(roll, pitch, yaw_start_);
        rotating_ = true;

        // Start rotation timer at 20 Hz
        rotate_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&pController::turn_and_look_for_art, this));
    }
}

void pController::move()
{
    // === 1. Exit if path is empty ===
    if (path_.poses.empty())
        return;

if (path_index_ >= path_.poses.size() && !sentry_mode_)
{
    if (!has_final_orientation_)
    {
        RCLCPP_WARN(this->get_logger(), "⚠️ No final orientation received yet. Cannot rotate.");
        return;
    }

    double final_dist = getDistanceError(path_.poses.back());  // position
    double yaw_now = getYaw(odo_.pose.pose.orientation);
    double yaw_goal = getYaw(final_goal_orientation_);
    double yaw_error = normalizeAngle(yaw_goal - yaw_now);

    if (!final_pose_reached_)
    {
        if (final_dist < tolerance)
        {
            final_pose_reached_ = true;
            RCLCPP_INFO(this->get_logger(), "📍 Final position reached.");
        }
        else
        {
            return;  // Still moving
        }
    }

    // Final rotation logic
    if (std::abs(yaw_error) > 0.1)
    {
        correcting_final_orientation_ = true;

        geometry_msgs::msg::Twist rotate_msg;
        rotate_msg.angular.z = Kp_angular * yaw_error;

        if (std::abs(rotate_msg.angular.z) < 0.1)
            rotate_msg.angular.z = 0.1 * (rotate_msg.angular.z > 0 ? 1 : -1);

        cmd_vel_pub_->publish(rotate_msg);
        return;
    }
    else if (correcting_final_orientation_)
    {
        RCLCPP_INFO(this->get_logger(), "✅ Final orientation corrected.");
        correcting_final_orientation_ = false;

        // Don't return here! Let the function continue to reach the publisher
    }

    // Final stop and publish
    RCLCPP_INFO(this->get_logger(), "🎯 All goals complete.");
    geometry_msgs::msg::Twist stop_msg;
    cmd_vel_pub_->publish(stop_msg);

    path_index_ = 0;
    path_.poses.clear();
    final_pose_reached_ = false;
    has_final_orientation_ = false;

    if (going_to_artwork_) 
    {
        std_msgs::msg::Bool msg;
        msg.data = true;
        RCLCPP_INFO(this->get_logger(), "📢 Publishing reached_artwork");
        reached_artwork_pub_->publish(msg);
        going_to_artwork_ = false;
    }

    return;
}





    if (sentry_mode_ && interrupted_)
    {

        RCLCPP_INFO(this->get_logger(), "Interrupted");

        geometry_msgs::msg::Twist stop_msg;
        cmd_vel_pub_->publish(stop_msg);

        path_index_ = 0;
        path_.poses.clear();
        sentry_mode_ = false;
        return;
    }

    double dist = getDistanceError(path_.poses.at(path_index_));

    // === 5. If goal is *inside* obstacle (not just obstructed), remove it ===
    // if (dist < 0.6 && obstacleavoidance_.isGoalInsideObstacle(getOdometry(), path_.poses.at(path_index_).pose.position))
    // {
    //     RCLCPP_WARN(this->get_logger(), "Goal %zu is inside obstacle. Deleting...", path_index_ + 1);
    //     goals_in_obstacles_++;
    //     path_.poses.erase(path_.poses.begin() + path_index_);
    //     return; // Skip movement for this tick
    // }

    if (dist < tolerance)
    {
        RCLCPP_INFO(this->get_logger(), "Reached goal %zu", path_index_ + 1);
        path_index_++;

        geometry_msgs::msg::Twist stop_msg;
        cmd_vel_pub_->publish(stop_msg);

        new_goal_targeted_ = true;
        obstruction_handled = false;
        going_to_safe_goal = false;
        return;
    }

    // === 7. Movement command ===
    double angle = getAngularError(path_.poses.at(path_index_));

    geometry_msgs::msg::Twist cmd_msg;
    cmd_msg.linear.x = Kp_linear * dist;
    cmd_msg.angular.z = Kp_angular * angle;

    // Clamp speeds
    if (cmd_msg.linear.x > 0.12)
        cmd_msg.linear.x = 0.12;

    if (cmd_msg.linear.x < 0.03)
        cmd_msg.linear.x = 0.03;

    // Smooth motion
    // static geometry_msgs::msg::Twist prev_cmd;
    // const double SMOOTHING_ALPHA = 0.9;
    // cmd_msg.linear.x = SMOOTHING_ALPHA * cmd_msg.linear.x + (1 - SMOOTHING_ALPHA) * prev_cmd.linear.x;
    // cmd_msg.angular.z = SMOOTHING_ALPHA * cmd_msg.angular.z + (1 - SMOOTHING_ALPHA) * prev_cmd.angular.z;
    // prev_cmd = cmd_msg;

    cmd_vel_pub_->publish(cmd_msg);
}

void pController::dancey_dance()
{
    if (!dancing_)
        return;

    // Get current yaw
    tf2::Quaternion q(
        odo_.pose.pose.orientation.x,
        odo_.pose.pose.orientation.y,
        odo_.pose.pose.orientation.z,
        odo_.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw_now;
    m.getRPY(roll, pitch, yaw_now);

    // First step: initialize yaw tracking
    if (first_dance_step_)
    {
        yaw_dance_last_ = yaw_now;
        first_dance_step_ = false;
    }

    // Compute delta with wrap-around
    double delta = yaw_now - yaw_dance_last_;
    if (delta > M_PI)
        delta -= 2 * M_PI;
    if (delta < -M_PI)
        delta += 2 * M_PI;

    yaw_dance_accumulated_ += std::abs(delta);
    yaw_dance_last_ = yaw_now;

    // RCLCPP_INFO(this->get_logger(),
    //     "Dance stage %d | delta: %.2f | rotated: %.2f rad",
    //     dance_stage_, delta, yaw_dance_accumulated_);

    // Rotation target: 30 degrees = π/6
    const double TARGET_ROTATION = M_PI / 8;

    if (yaw_dance_accumulated_ >= TARGET_ROTATION)
    {
        // Stage complete → reverse direction
        dance_stage_ = (dance_stage_ + 1) % 2;
        yaw_dance_accumulated_ = 0.0;
        first_dance_step_ = true;

        if (dance_stage_ == 0)
        {
            dance_cycles_++;
        }

        // After 3 full left-right cycles
        if (dance_cycles_ >= 3)
        {
            geometry_msgs::msg::Twist stop;
            cmd_vel_pub_->publish(stop);
            dance_timer_->cancel();
            dancing_ = false;
            RCLCPP_INFO(this->get_logger(), "🕺 Dance complete!");
            return;
        }
    }

    // Command angular velocity
    geometry_msgs::msg::Twist twist;
    twist.angular.z = (dance_stage_ == 0) ? 0.5 : -0.5;
    cmd_vel_pub_->publish(twist);
}

void pController::turn_and_look_for_art()
{
    if (!rotating_)
        return;

    // Get current yaw
    tf2::Quaternion q_now(
        odo_.pose.pose.orientation.x,
        odo_.pose.pose.orientation.y,
        odo_.pose.pose.orientation.z,
        odo_.pose.pose.orientation.w);
    tf2::Matrix3x3 m_now(q_now);
    double roll, pitch, yaw_now;
    m_now.getRPY(roll, pitch, yaw_now);

    // Initialize yaw_last_ only once
    if (first_rotation_step_)
    {
        yaw_last_ = yaw_now;
        first_rotation_step_ = false;
    }

    // Compute delta yaw with wrap-around handling
    double delta = yaw_now - yaw_last_;
    if (delta > M_PI)
        delta -= 2 * M_PI;
    if (delta < -M_PI)
        delta += 2 * M_PI;

    yaw_accumulated_ += std::abs(delta);
    yaw_last_ = yaw_now;

    // RCLCPP_INFO(this->get_logger(),
    // "Rotated so far: %.2f rad | yaw_now: %.2f",
    // yaw_accumulated_, yaw_now);

    if (yaw_accumulated_ >= 2 * M_PI)
    {
        geometry_msgs::msg::Twist stop;
        cmd_vel_pub_->publish(stop);
        rotate_timer_->cancel();
        rotating_ = false;
        yaw_accumulated_ = 0.0;
        first_rotation_step_ = true;
        RCLCPP_INFO(this->get_logger(), "Rotation complete.");
        return;
    }

    if (service_call_pending_)
        return;

    service_call_pending_ = true;

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;

    auto future = client_->async_send_request(request,
                                              [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture result)
                                              {
                                                  service_call_pending_ = false;
                                                  if (result.get()->success)
                                                  {
                                                      RCLCPP_INFO(this->get_logger(), "Service responded: SUCCESS - %s", result.get()->message.c_str());
                                                      geometry_msgs::msg::Twist stop;
                                                      cmd_vel_pub_->publish(stop);
                                                      rotate_timer_->cancel();
                                                      rotating_ = false;
                                                      yaw_accumulated_ = 0.0;
                                                      first_rotation_step_ = true;
                                                      RCLCPP_INFO(this->get_logger(), "Artwork matched.");
                                                      std_msgs::msg::Bool msg;
                                                      msg.data = true;
                                                      reached_artwork_pub_->publish(msg);
                                                  }
                                                  else
                                                  {
                                                      RCLCPP_WARN(this->get_logger(), "Service responded: FAILURE - %s", result.get()->message.c_str());
                                                  }
                                              });

    // Keep turning
    geometry_msgs::msg::Twist twist;
    twist.angular.z = 0.3;
    cmd_vel_pub_->publish(twist);
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
    //     tf2::Quaternion q(
    //     msg->pose.pose.orientation.x,
    //     msg->pose.pose.orientation.y,
    //     msg->pose.pose.orientation.z,
    //     msg->pose.pose.orientation.w);
    // tf2::Matrix3x3 m(q);
    // double roll, pitch, current_theta;
    // m.getRPY(roll, pitch, current_theta);
    // RCLCPP_INFO(this->get_logger(), "x: %.2f | y: %.2f | theta: %.2f", msg->pose.pose.position.x, msg->pose.pose.position.y, current_theta);
    odo_ = *msg;
}

void pController::path_callback(const std::shared_ptr<nav_msgs::msg::Path> path)
{

    path_ = *path;

    if (path_.poses.size() > 0)
    {
        RCLCPP_INFO(this->get_logger(), "Recieved Path");
    }
}

nav_msgs::msg::Odometry pController::getOdometry(void)
{

    nav_msgs::msg::Odometry pose = odo_;
    return pose;
}

double pController::getYaw(const geometry_msgs::msg::Quaternion &q)
{
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    return yaw;
}

double pController::normalizeAngle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}


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

// PID

// #include "p_controller.hpp"

// pController::pController(ObstacleAvoidance &obstacleavoidance) : Node("p_controller"), tolerance(0.1), obstacleavoidance_(obstacleavoidance)
// {

//     cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

//     // Create a subscriber to get odometry data
//     odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//         "/odom", 10, std::bind(&pController::odom_callback, this, std::placeholders::_1));

//     path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
//         "computed_path", 10, std::bind(&pController::path_callback, this, std::placeholders::_1));

//     mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
//         "/robot_mode", 10, std::bind(&pController::mode_callback, this, std::placeholders::_1));

//     interrupt_sub_ = this->create_subscription<std_msgs::msg::Bool>(
//         "/interrupt_signal", 10, std::bind(&pController::interrupt_callback, this, std::placeholders::_1));

//     timer_ = this->create_wall_timer(
//         std::chrono::milliseconds(100),
//         std::bind(&pController::move, this));

//     pub_table_marker = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", 1000);

//     // Constructor initialization
// Kp_angular = 0.6;
// Ki_angular = 0.0;
// Kd_angular = 0.5;

// Kp_linear = 1.0;
// Ki_linear = 0.0;
// Kd_linear = 0.1;

// last_time = this->now();

// }

// void pController::interrupt_callback(const std_msgs::msg::Bool::SharedPtr msg)
// {
//     interrupted_ = msg->data;
// }

// void pController::mode_callback(const std_msgs::msg::Int32::SharedPtr msg)
// {
//     received_int_ = msg->data;
//     RCLCPP_INFO(this->get_logger(), "Received control mode: %d", received_int_);

//     if (received_int_ == 1)
//     {
//         // sentry_mode_ = true;
//         RCLCPP_INFO(this->get_logger(), "Sentry Mode Activated");
//     }

//     else if (received_int_ == 2)
//     {
//         sentry_mode_ = false;
//         RCLCPP_INFO(this->get_logger(), "💃 Dance mode activated!");

//         dancing_ = true;
//         dance_cycles_ = 0;
//         dance_stage_ = 0; // 0 = rotate left, 1 = rotate right
//         yaw_dance_accumulated_ = 0.0;
//         first_dance_step_ = true;

//         dance_timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(50),
//             std::bind(&pController::dancey_dance, this));
//     }

//     else if (received_int_ == 3)
//     {
//         sentry_mode_ = false;
//         RCLCPP_INFO(this->get_logger(), "Starting 360° rotation...");

//         tf2::Quaternion q_start(
//             odo_.pose.pose.orientation.x,
//             odo_.pose.pose.orientation.y,
//             odo_.pose.pose.orientation.z,
//             odo_.pose.pose.orientation.w);
//         tf2::Matrix3x3 m_start(q_start);
//         double roll, pitch;
//         m_start.getRPY(roll, pitch, yaw_start_);
//         rotating_ = true;

//         // Start rotation timer at 20 Hz
//         rotate_timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(50),
//             std::bind(&pController::turn_and_look_for_art, this));
//     }
// }
// void pController::move()
// {
//     if (path_.poses.empty()) return;

//     if (path_index_ >= path_.poses.size() && !sentry_mode_)
//     {
//         RCLCPP_INFO(this->get_logger(), "All waypoints reached.");
//         RCLCPP_INFO(this->get_logger(), "Goals inside obstacles: %d", goals_in_obstacles_);

//         geometry_msgs::msg::Twist stop_msg;
//         cmd_vel_pub_->publish(stop_msg);

//         path_index_ = 0;
//         path_.poses.clear();
//         return;
//     }

//     rclcpp::Time now = this->now();
//     double dt = (now - last_time).seconds();
//     if (dt == 0.0) return;

//     double dist = getDistanceError(path_.poses.at(path_index_));
//     double angle = getAngularError(path_.poses.at(path_index_));

//     // === PID Control Logic ===
//     double linear_error = dist;
//     double angular_error = angle;

//     sum_linear_error += linear_error * dt;
//     sum_angular_error += angular_error * dt;

//     double linear_derivative = (linear_error - prev_linear_error) / dt;
//     double angular_derivative = (angular_error - prev_angular_error) / dt;

//     double linear_output = Kp_linear * linear_error + Ki_linear * sum_linear_error + Kd_linear * linear_derivative;
//     double angular_output = Kp_angular * angular_error + Ki_angular * sum_angular_error + Kd_angular * angular_derivative;

//     prev_linear_error = linear_error;
//     prev_angular_error = angular_error;
//     last_time = now;

//     // Clamp speeds
//     if (linear_output > 0.12) linear_output = 0.12;
//     if (linear_output < 0.03) linear_output = 0.03;

//     // === Check goal proximity ===
//     if (dist < tolerance)
//     {
//         RCLCPP_INFO(this->get_logger(), "Reached goal %zu", path_index_ + 1);
//         path_index_++;

//         geometry_msgs::msg::Twist stop_msg;
//         cmd_vel_pub_->publish(stop_msg);

//         new_goal_targeted_ = true;
//         obstruction_handled = false;
//         going_to_safe_goal = false;
//         return;
//     }

//     geometry_msgs::msg::Twist cmd_msg;
//     cmd_msg.linear.x = linear_output;

// //     const double ANGLE_THRESHOLD = 0.2;  // ~11 degrees

// // // Rotate first if heading is off
// // if (std::abs(angle) > ANGLE_THRESHOLD)
// // {
// //     cmd_msg.linear.x = 0.0;  // No forward movement until aligned
// // }
// // else
// // {
// //     cmd_msg.linear.x = linear_output;
// // }

// // cmd_msg.angular.z = angular_output;

//     cmd_msg.angular.z = angular_output;

//     cmd_vel_pub_->publish(cmd_msg);
// }

// void pController::dancey_dance()
// {
//     if (!dancing_)
//         return;

//     // Get current yaw
//     tf2::Quaternion q(
//         odo_.pose.pose.orientation.x,
//         odo_.pose.pose.orientation.y,
//         odo_.pose.pose.orientation.z,
//         odo_.pose.pose.orientation.w);
//     tf2::Matrix3x3 m(q);
//     double roll, pitch, yaw_now;
//     m.getRPY(roll, pitch, yaw_now);

//     // First step: initialize yaw tracking
//     if (first_dance_step_)
//     {
//         yaw_dance_last_ = yaw_now;
//         first_dance_step_ = false;
//     }

//     // Compute delta with wrap-around
//     double delta = yaw_now - yaw_dance_last_;
//     if (delta > M_PI)
//         delta -= 2 * M_PI;
//     if (delta < -M_PI)
//         delta += 2 * M_PI;

//     yaw_dance_accumulated_ += std::abs(delta);
//     yaw_dance_last_ = yaw_now;

//     // RCLCPP_INFO(this->get_logger(),
//     //     "Dance stage %d | delta: %.2f | rotated: %.2f rad",
//     //     dance_stage_, delta, yaw_dance_accumulated_);

//     // Rotation target: 30 degrees = π/6
//     const double TARGET_ROTATION = M_PI / 8;

//     if (yaw_dance_accumulated_ >= TARGET_ROTATION)
//     {
//         // Stage complete → reverse direction
//         dance_stage_ = (dance_stage_ + 1) % 2;
//         yaw_dance_accumulated_ = 0.0;
//         first_dance_step_ = true;

//         if (dance_stage_ == 0)
//         {
//             dance_cycles_++;
//         }

//         // After 3 full left-right cycles
//         if (dance_cycles_ >= 3)
//         {
//             geometry_msgs::msg::Twist stop;
//             cmd_vel_pub_->publish(stop);
//             dance_timer_->cancel();
//             dancing_ = false;
//             RCLCPP_INFO(this->get_logger(), "🕺 Dance complete!");
//             return;
//         }
//     }

//     // Command angular velocity
//     geometry_msgs::msg::Twist twist;
//     twist.angular.z = (dance_stage_ == 0) ? 0.5 : -0.5;
//     cmd_vel_pub_->publish(twist);
// }

// void pController::turn_and_look_for_art()
// {
//     if (!rotating_)
//         return;

//     // Get current yaw
//     tf2::Quaternion q_now(
//         odo_.pose.pose.orientation.x,
//         odo_.pose.pose.orientation.y,
//         odo_.pose.pose.orientation.z,
//         odo_.pose.pose.orientation.w);
//     tf2::Matrix3x3 m_now(q_now);
//     double roll, pitch, yaw_now;
//     m_now.getRPY(roll, pitch, yaw_now);

//     // Initialize yaw_last_ only once
//     if (first_rotation_step_)
//     {
//         yaw_last_ = yaw_now;
//         first_rotation_step_ = false;
//     }

//     // Compute delta yaw with wrap-around handling
//     double delta = yaw_now - yaw_last_;
//     if (delta > M_PI)
//         delta -= 2 * M_PI;
//     if (delta < -M_PI)
//         delta += 2 * M_PI;

//     yaw_accumulated_ += std::abs(delta);
//     yaw_last_ = yaw_now;

//     // RCLCPP_INFO(this->get_logger(),
//     // "Rotated so far: %.2f rad | yaw_now: %.2f",
//     // yaw_accumulated_, yaw_now);

//     if (yaw_accumulated_ >= 2 * M_PI || interrupted_)
//     {
//         geometry_msgs::msg::Twist stop;
//         cmd_vel_pub_->publish(stop);
//         rotate_timer_->cancel();
//         rotating_ = false;
//         yaw_accumulated_ = 0.0;
//         first_rotation_step_ = true;
//         RCLCPP_INFO(this->get_logger(), "Rotation complete or interrupted.");
//         return;
//     }

//     // Keep turning
//     geometry_msgs::msg::Twist twist;
//     twist.angular.z = 0.3;
//     cmd_vel_pub_->publish(twist);
// }

// double pController::getDistanceError(const geometry_msgs::msg::PoseStamped &goal)
// {
//     double current_x = getOdometry().pose.pose.position.x;
//     double current_y = getOdometry().pose.pose.position.y;

//     double error_x = goal.pose.position.x - current_x;
//     double error_y = goal.pose.position.y - current_y;
//     double distance_error = std::sqrt(error_x * error_x + error_y * error_y);

//     return distance_error;
// }

// double pController::getAngularError(const geometry_msgs::msg::PoseStamped &goal)
// {
//     double error_x = goal.pose.position.x - getOdometry().pose.pose.position.x;
//     double error_y = goal.pose.position.y - getOdometry().pose.pose.position.y;

//     double desired_theta = std::atan2(error_y, error_x);

//     // Get current robot orientation (convert quaternion to yaw)
//     tf2::Quaternion q(
//         getOdometry().pose.pose.orientation.x,
//         getOdometry().pose.pose.orientation.y,
//         getOdometry().pose.pose.orientation.z,
//         getOdometry().pose.pose.orientation.w);
//     tf2::Matrix3x3 m(q);
//     double roll, pitch, current_theta;
//     m.getRPY(roll, pitch, current_theta);

//     // Compute angular error
//     double angular_error = desired_theta - current_theta;
//     while (angular_error > M_PI)
//         angular_error -= 2 * M_PI;
//     while (angular_error < -M_PI)
//         angular_error += 2 * M_PI;

//     return angular_error;
// }

// void pController::odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
// {
//     // RCLCPP_INFO(this->get_logger(), "x: %.2f | y: %.2f", msg->pose.pose.position.x, msg->pose.pose.position.y);
//     odo_ = *msg;
// }

// void pController::path_callback(const std::shared_ptr<nav_msgs::msg::Path> path)
// {

//     path_ = *path;

//     if (path_.poses.size() > 0)
//     {
//         RCLCPP_INFO(this->get_logger(), "Recieved Path");
//     }
// }

// nav_msgs::msg::Odometry pController::getOdometry(void)
// {

//     nav_msgs::msg::Odometry pose = odo_;
//     return pose;
// }

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);

//     auto avoidance_node = std::make_shared<ObstacleAvoidance>();
//     auto controller_node = std::make_shared<pController>(*avoidance_node);

//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(avoidance_node);
//     executor.add_node(controller_node);
//     executor.spin();

//     rclcpp::shutdown();
//     return 0;
// }

// // consider using moveIT
// // wrapping the angle constricting to -180 to 180
