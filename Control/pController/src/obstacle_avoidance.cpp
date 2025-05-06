#include "obstacle_avoidance.hpp"

ObstacleAvoidance::ObstacleAvoidance() : Node("obstacle_avoidance")
{

    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&ObstacleAvoidance::laserCallback, this, std::placeholders::_1));

    sub_odo_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 100, std::bind(&ObstacleAvoidance::odomCallback, this, std::placeholders::_1));

}

std::vector<geometry_msgs::msg::Point> ObstacleAvoidance::getLaserPoints() const
{
    std::vector<geometry_msgs::msg::Point> points;

    for (size_t i = 0; i < scan_.ranges.size(); ++i)
    {
        float r = scan_.ranges[i];
        if (std::isfinite(r) &&
            r > scan_.range_min &&
            r < scan_.range_max)
        {

            float angle = scan_.angle_min + i * scan_.angle_increment;

            geometry_msgs::msg::Point p_local;
            p_local.x = r * std::cos(angle);
            p_local.y = r * std::sin(angle);
            p_local.z = 0;

            geometry_msgs::msg::Point p_global = localToGlobal(odo_, p_local);
            points.push_back(p_global);
        }
    }

    return points;
}

// bool ObstacleAvoidance::isGoalObstructed(const nav_msgs::msg::Odometry &odom, const geometry_msgs::msg::Point &goal)
// {
//     std::vector<geometry_msgs::msg::Point> points = getLaserPoints();

//     // Get robot's orientation
//     tf2::Quaternion q(
//         odom.pose.pose.orientation.x,
//         odom.pose.pose.orientation.y,
//         odom.pose.pose.orientation.z,
//         odom.pose.pose.orientation.w);
//     tf2::Matrix3x3 m(q);
//     double roll, pitch, yaw;
//     m.getRPY(roll, pitch, yaw);  // yaw = robot’s heading

//     const double FRONT_ANGLE_CONE = M_PI; // 90 degrees total (±45°)
//     const double COLLISION_DISTANCE = 0.35;    // danger zone

//     for (const auto &point : points)
//     {
//         double dx = point.x - odom.pose.pose.position.x;
//         double dy = point.y - odom.pose.pose.position.y;
//         double distance = std::hypot(dx, dy);

//         if (distance > COLLISION_DISTANCE)
//             continue;

//         double angle_to_point = std::atan2(dy, dx);
//         double angle_diff = angle_to_point - yaw;

//         // Normalize to [-π, π]
//         while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
//         while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

//         // Check if within frontal cone
//         if (std::fabs(angle_diff) < FRONT_ANGLE_CONE / 2)
//         {
//             RCLCPP_WARN(rclcpp::get_logger("ObstacleAvoidance"), 
//                         "collision avoidance kicking in, Angle: %.1f°, Distance: %.2f m",
//                         angle_diff * 180.0 / M_PI, distance);
//             return true;
//         }
//     }

//     return false;
// }


// bool ObstacleAvoidance::isGoalObstructed(const nav_msgs::msg::Odometry &odom, const geometry_msgs::msg::Point &goal)
// {
//     std::vector<geometry_msgs::msg::Point> points = getLaserPoints();

//     const double COLLISION_DISTANCE = 0.325;    // danger zone

//     for (const auto &point : points)
//     {
//         double dx = point.x - odom.pose.pose.position.x;
//         double dy = point.y - odom.pose.pose.position.y;
//         double distance = std::hypot(dx, dy);

//         if (distance < COLLISION_DISTANCE){
//             RCLCPP_WARN(rclcpp::get_logger("ObstacleAvoidance"), "collision avoidance kicking in");
//             return true;
//         }
//     }

//     return false;
// }

bool ObstacleAvoidance::isGoalObstructed(const nav_msgs::msg::Odometry &odom, const geometry_msgs::msg::Point &goal)
{
    std::vector<geometry_msgs::msg::Point> points = getLaserPoints();

    const double FRONT_COLLISION_DISTANCE = 0.5;
    const double SIDE_COLLISION_DISTANCE = 0.1;
    const double FRONT_ANGLE_THRESHOLD = M_PI / 5;  // ±45 degrees

    geometry_msgs::msg::Point robot_pos = odom.pose.pose.position;

    // Extract robot yaw from orientation quaternion
    tf2::Quaternion q(
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    for (const auto &point : points)
    {
        double dx = point.x - robot_pos.x;
        double dy = point.y - robot_pos.y;
        double distance = std::hypot(dx, dy);
        double angle_to_point = std::atan2(dy, dx);
        double relative_angle = shortestAngularDistance(yaw, angle_to_point);

        double threshold = std::abs(relative_angle) < FRONT_ANGLE_THRESHOLD
                           ? FRONT_COLLISION_DISTANCE
                           : SIDE_COLLISION_DISTANCE;

        if (distance < threshold) {
            RCLCPP_WARN(rclcpp::get_logger("ObstacleAvoidance"),
                        "⚠️ Obstacle at angle %.2f, distance %.2f", relative_angle, distance);
            return true;
        }
    }

    return false;
}



double ObstacleAvoidance::shortestAngularDistance(double from, double to)
{
    double result = to - from;
    while (result > M_PI) result -= 2.0 * M_PI;
    while (result < -M_PI) result += 2.0 * M_PI;
    return result;
}



// bool ObstacleAvoidance::isGoalInsideObstacle(const nav_msgs::msg::Odometry &odom, const geometry_msgs::msg::Point &goal) 
// {
//     std::vector<geometry_msgs::msg::Point> points = getLaserPoints();

//     const double DISTANCE_THRESHOLD = 0.2;           // Max distance between laser point and goal
//     const double ANGLE_THRESHOLD = M_PI / 6.0;       // ±30 degrees cone

//     geometry_msgs::msg::Point robot_pos = odom.pose.pose.position;

//     // Direction to the goal from the robot
//     double dx_goal = goal.x - robot_pos.x;
//     double dy_goal = goal.y - robot_pos.y;
//     double goal_distance = std::hypot(dx_goal, dy_goal);
//     double angle_to_goal = std::atan2(dy_goal, dx_goal);

//     for (const auto &point : points)
//     {
//         // Vector from robot to current point
//         double dx = point.x - robot_pos.x;
//         double dy = point.y - robot_pos.y;
//         double distance_to_point = std::hypot(dx, dy);
//         double angle_to_point = std::atan2(dy, dx);

//         // Step 1: Check if the point is within angular range of the goal direction
//         double angle_diff = angle_to_point - angle_to_goal;
//         while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
//         while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

//         if (std::fabs(angle_diff) > ANGLE_THRESHOLD)
//             continue;  // Ignore points outside the angular range

//         // Step 2: Check if the point lies in front of the goal (closer to robot than the goal is)
//         if (distance_to_point > goal_distance + 0.2)
//             continue;

//         // Step 3: Check if the point is close enough to the goal
//         double distance_to_goal = std::hypot(point.x - goal.x, point.y - goal.y);
//         if (distance_to_goal < DISTANCE_THRESHOLD)
//         {
//             RCLCPP_WARN(rclcpp::get_logger("ObstacleAvoidance"),
//                         "Goal at (%.2f, %.2f) is inside an obstacle!",
//                         goal.x, goal.y);
//             return true;
//         }
//     }

//     return false;
// }


// bool ObstacleAvoidance::isGoalInsideObstacle(const nav_msgs::msg::Odometry &odom, const geometry_msgs::msg::Point &goal)
// {
//     const double THRESHOLD = 0.25; // Distance threshold to consider goal inside an obstacle
//     geometry_msgs::msg::Point robot_pos = odom.pose.pose.position;
//     std::vector<geometry_msgs::msg::Point> laser_points = getLaserPoints();

//     for (const auto &pt : laser_points)
//     {
//         double dist = std::hypot(pt.x - goal.x, pt.y - goal.y);
//         if (dist < THRESHOLD)
//         {
//             RCLCPP_WARN(rclcpp::get_logger("ObstacleAvoidance"),
//                         "Goal at (%.2f, %.2f) is inside an obstacle! (Laser point at %.2f, %.2f, dist: %.2f)",
//                         goal.x, goal.y, pt.x, pt.y, dist);
//             return true;
//         }
//     }

//     return false;
// }

bool ObstacleAvoidance::isGoalInsideObstacle(const nav_msgs::msg::Odometry &odom, const geometry_msgs::msg::Point &goal)
{
    const double DISTANCE_THRESHOLD = 0.25;           // Max distance between laser point and goal
    const double ANGLE_THRESHOLD = M_PI / 6.0;        // ±30 degrees cone

    geometry_msgs::msg::Point robot_pos = odom.pose.pose.position;

    // Compute angle from robot to goal
    double dx_goal = goal.x - robot_pos.x;
    double dy_goal = goal.y - robot_pos.y;
    double angle_to_goal = std::atan2(dy_goal, dx_goal);
    double dist_to_goal = std::hypot(dx_goal, dy_goal);

    for (const auto &point : getLaserPoints())  // Already global
    {
        double dx = point.x - robot_pos.x;
        double dy = point.y - robot_pos.y;
        double angle_to_point = std::atan2(dy, dx);
        double angle_diff = angle_to_point - angle_to_goal;

        // Normalize to [-pi, pi]
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        // Check if point is within a cone toward the goal
        if (std::fabs(angle_diff) > ANGLE_THRESHOLD)
            continue;

        // Check if laser point is close to goal
        double distance_to_goal = std::hypot(point.x - goal.x, point.y - goal.y);
        if (distance_to_goal < DISTANCE_THRESHOLD)
        {
            RCLCPP_WARN(rclcpp::get_logger("ObstacleAvoidance"),
                        "🚫 Goal at (%.2f, %.2f) appears inside obstacle (Laser point at %.2f, %.2f, dist %.2f)",
                        goal.x, goal.y, point.x, point.y, distance_to_goal);
            return true;
        }
    }

    return false;
}




geometry_msgs::msg::Twist ObstacleAvoidance::adjustVelocity(const geometry_msgs::msg::Twist &original_cmd)
{
    return original_cmd;
}



// geometry_msgs::msg::PoseStamped ObstacleAvoidance::suggestNewGoal(
//     const geometry_msgs::msg::PoseStamped &original_goal,
//     const geometry_msgs::msg::PoseStamped &next_goal,
//     double attraction,
//     double repulsion,
//     double goal_step)
// {
//     geometry_msgs::msg::Point robot_pos = odo_.pose.pose.position;
//     std::vector<geometry_msgs::msg::Point> obstacles = getLaserPoints();

//     double dx = original_goal.pose.position.x - robot_pos.x;
//     double dy = original_goal.pose.position.y - robot_pos.y;

//     double fx_att = attraction * dx;
//     double fy_att = attraction * dy;

//     double fx_rep = 0.0, fy_rep = 0.0;
//     for (const auto &obs : obstacles) {
//         double ox = obs.x - robot_pos.x;
//         double oy = obs.y - robot_pos.y;
//         double d = std::hypot(ox, oy);
//         if (d < 1e-2 || d > 1.0) continue;

//         double rep_factor = repulsion * (1.0 / d - 1.0) / (d * d);
//         fx_rep -= rep_factor * (ox / d);
//         fy_rep -= rep_factor * (oy / d);
//     }

//     // Combine forces
//     double fx = fx_att + fx_rep;
//     double fy = fy_att + fy_rep;
//     double norm = std::hypot(fx, fy);

//     // If path is blocked or force is weak, do 45-degree side step
//     if (isPathToGoalObstructed(robot_pos, original_goal.pose.position, 0.2, 0.1) || norm < 1e-3) {
//         RCLCPP_WARN(rclcpp::get_logger("obstacle_avoidance"), "⚠️ Obstacle detected. Executing 45° sidestep...");

//         double angle_45 = M_PI / 4.0;
//         double rotated_fx = fx_att * std::cos(angle_45) - fy_att * std::sin(angle_45);
//         double rotated_fy = fx_att * std::sin(angle_45) + fy_att * std::cos(angle_45);
//         double rotated_norm = std::hypot(rotated_fx, rotated_fy);

//         geometry_msgs::msg::PoseStamped sidestep_goal = original_goal;
//         sidestep_goal.pose.position.x = robot_pos.x + rotated_fx / rotated_norm * goal_step;
//         sidestep_goal.pose.position.y = robot_pos.y + rotated_fy / rotated_norm * goal_step;
//         return sidestep_goal;
//     }

//     // Otherwise proceed with combined force vector
//     geometry_msgs::msg::PoseStamped new_goal = original_goal;
//     new_goal.pose.position.x = robot_pos.x + fx / norm * goal_step;
//     new_goal.pose.position.y = robot_pos.y + fy / norm * goal_step;
//     return new_goal;
// }

bool ObstacleAvoidance::collisionIminent(const nav_msgs::msg::Odometry &odom){

    geometry_msgs::msg::Point robot_pos = odom.pose.pose.position;
    std::vector<geometry_msgs::msg::Point> points = getLaserPoints();
    for(const auto& point : points){
        double dist = std::hypot(robot_pos.x - point.x, robot_pos.y - point.y);
        if(dist <= 0.15){
            return true;
        }
    }

    return false;
}


bool ObstacleAvoidance::collisionTooClose(const nav_msgs::msg::Odometry &odom, double clearance)
{
    geometry_msgs::msg::Point robot_pos = odom.pose.pose.position;
    std::vector<geometry_msgs::msg::Point> points = getLaserPoints();
    for (const auto &point : points)
    {
        double dist = std::hypot(robot_pos.x - point.x, robot_pos.y - point.y);
        if (dist < clearance)
            return true;
    }
    return false;
}


geometry_msgs::msg::PoseStamped ObstacleAvoidance::suggestNewGoal(
    const geometry_msgs::msg::PoseStamped &original_goal,
    const geometry_msgs::msg::PoseStamped &next_goal,
    double attraction,
    double repulsion,
    double goal_step)
{
    geometry_msgs::msg::Point robot_pos = odo_.pose.pose.position;
    std::vector<geometry_msgs::msg::Point> obstacles = getLaserPoints();

    // --- Compute robot yaw (heading) ---
    tf2::Quaternion q(
        odo_.pose.pose.orientation.x,
        odo_.pose.pose.orientation.y,
        odo_.pose.pose.orientation.z,
        odo_.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // --- Attractive force towards the goal ---
    double dx = original_goal.pose.position.x - robot_pos.x;
    double dy = original_goal.pose.position.y - robot_pos.y;
    double dist_to_goal = std::hypot(dx, dy);

    double fx_att = attraction * dx;
    double fy_att = attraction * dy;

    // --- Perpendicular direction for repulsion ---
    double goal_dir_x = dx / (dist_to_goal + 1e-6);
    double goal_dir_y = dy / (dist_to_goal + 1e-6);
    double perp_dir_x = -goal_dir_y;
    double perp_dir_y = goal_dir_x;

    double fx_rep = 0.0;
    double fy_rep = 0.0;

    for (const auto &obs : obstacles) {
        double ox = obs.x - robot_pos.x;
        double oy = obs.y - robot_pos.y;
        double d = std::hypot(ox, oy);
        if (d < 1e-2 || d > 1.0) continue;

        double rep_factor = repulsion * (1.0 / d - 1.0) / (d * d);
        double dir_x = -ox / d;
        double dir_y = -oy / d;

        double dot = dir_x * perp_dir_x + dir_y * perp_dir_y;
        fx_rep += rep_factor * dot * perp_dir_x;
        fy_rep += rep_factor * dot * perp_dir_y;
    }

    // Combine attractive and repulsive forces
    double fx = fx_att + fx_rep;
    double fy = fy_att + fy_rep;
    double norm = std::hypot(fx, fy);

    // Sidestep if blocked
    if (isPathToGoalObstructed(robot_pos, original_goal.pose.position, 0.2, 0.1) || norm < 1e-3) {
        RCLCPP_WARN(rclcpp::get_logger("obstacle_avoidance"), "⚠️ Obstacle detected. Executing 45° sidestep...");

        double angle_45 = M_PI / 4.0;
        double rotated_fx = fx_att * std::cos(angle_45) - fy_att * std::sin(angle_45);
        double rotated_fy = fx_att * std::sin(angle_45) + fy_att * std::cos(angle_45);
        double rotated_norm = std::hypot(rotated_fx, rotated_fy);

        geometry_msgs::msg::PoseStamped sidestep_goal = original_goal;
        sidestep_goal.pose.position.x = robot_pos.x + rotated_fx / rotated_norm * goal_step;
        sidestep_goal.pose.position.y = robot_pos.y + rotated_fy / rotated_norm * goal_step;

        // Add forward offset of 0.4 m

        double noise = ((double)rand() / RAND_MAX - 0.5) * 0.1;  // ±0.05 rad
        double noisy_yaw = yaw + noise; 
        sidestep_goal.pose.position.x += 0.4 * std::cos(noisy_yaw);
        sidestep_goal.pose.position.y += 0.4 * std::sin(noisy_yaw);
        return sidestep_goal;
    }

    // Normal case: new goal based on potential field
    geometry_msgs::msg::PoseStamped new_goal = original_goal;
    new_goal.pose.position.x = robot_pos.x + fx / norm * goal_step;
    new_goal.pose.position.y = robot_pos.y + fy / norm * goal_step;

    // Add forward offset of 0.4 m
// Dynamic forward scanning to find safe goal
const double max_forward_dist = 0.6;
const double min_clearance = 0.2;
const double step_size = 0.05;
double best_dist = 0.0;

for (double step = step_size; step <= max_forward_dist; step += step_size) {
    double test_x = new_goal.pose.position.x + step * std::cos(yaw);
    double test_y = new_goal.pose.position.y + step * std::sin(yaw);

    bool is_clear = true;
    for (const auto& obs : obstacles) {
        double dist = std::hypot(test_x - obs.x, test_y - obs.y);
        if (dist < min_clearance) {
            is_clear = false;
            break;
        }
    }

    if (is_clear) {
        best_dist = step;
    } else {
        break; // Stop at the first obstacle
    }
}

new_goal.pose.position.x += best_dist * std::cos(yaw);
new_goal.pose.position.y += best_dist * std::sin(yaw);


    return new_goal;
}



// bool ObstacleAvoidance::isPathToGoalObstructed(const geometry_msgs::msg::Point& start,
//                                                const geometry_msgs::msg::Point& goal) const
// {
//     std::vector<geometry_msgs::msg::Point> laser_points = getLaserPoints();
//     const double STEP = 0.05;
//     const double OBSTACLE_THRESHOLD = 0.25;

//     double dx = goal.x - start.x;
//     double dy = goal.y - start.y;
//     double dist = std::hypot(dx, dy);
//     int steps = std::max(1, static_cast<int>(dist / STEP));

//     for (int i = 1; i <= steps; ++i)
//     {
//         double ratio = static_cast<double>(i) / steps;
//         double x = start.x + dx * ratio;
//         double y = start.y + dy * ratio;

//         for (const auto& obs : laser_points)
//         {
//             if (std::hypot(obs.x - x, obs.y - y) < OBSTACLE_THRESHOLD)
//                 return true;
//         }
//     }
//     return false;
// }



bool ObstacleAvoidance::isPathToGoalObstructed(const geometry_msgs::msg::Point& start,
                                               const geometry_msgs::msg::Point& goal, double front_dist, double side_dist) const
{
    // Define safety distances and angular thresholds
    const double SAFE_DIST_FRONT = front_dist;//0.325;
    const double SAFE_DIST_SIDE = side_dist;//0.15;
    const double FRONT_ANGLE = M_PI / 6.0;   // ±30°
    const double SIDE_ANGLE = M_PI / 2.0;    // ±90°

    // Extract robot's orientation (yaw) from odometry
    tf2::Quaternion q(
        odo_.pose.pose.orientation.x,
        odo_.pose.pose.orientation.y,
        odo_.pose.pose.orientation.z,
        odo_.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    geometry_msgs::msg::Point robot_pos = odo_.pose.pose.position;

    // Initialize blockage flags
    bool front_blocked = false;
    bool left_blocked = false;
    bool right_blocked = false;

    // Analyze each laser point
    for (const auto &point : getLaserPoints())
    {
        double dx = point.x - robot_pos.x;
        double dy = point.y - robot_pos.y;
        double angle = std::atan2(dy, dx) - yaw;
        double dist = std::hypot(dx, dy);

        // Normalize angle to [-π, π]
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;

        // Check for obstacles in front
        if (std::fabs(angle) < FRONT_ANGLE && dist < SAFE_DIST_FRONT)
            front_blocked = true;

        // Check for obstacles on the left
        if (angle > FRONT_ANGLE && angle < SIDE_ANGLE && dist < SAFE_DIST_SIDE)
            left_blocked = true;

        // Check for obstacles on the right
        if (angle < -FRONT_ANGLE && angle > -SIDE_ANGLE && dist < SAFE_DIST_SIDE)
            right_blocked = true;
    }

    // Log warnings if obstacles are detected
    if (front_blocked)
        RCLCPP_WARN(this->get_logger(), "⚠️ Obstacle ahead!");

    if (left_blocked || right_blocked)
        RCLCPP_WARN(this->get_logger(), "⚠️ Obstacle on the %s side!", left_blocked ? "left" : "right");

    // Return true if any path is obstructed
    return front_blocked || left_blocked || right_blocked;
}



bool ObstacleAvoidance::isGoalReachableDespiteObstruction(
    const geometry_msgs::msg::PoseStamped &goal,
    double max_distance,
    double max_angle_rad,
    double clearance_radius) const
{
    geometry_msgs::msg::Point robot_pos = odo_.pose.pose.position;

    // Compute distance and angle to goal
    double dx = goal.pose.position.x - robot_pos.x;
    double dy = goal.pose.position.y - robot_pos.y;
    double dist_to_goal = std::hypot(dx, dy);
    double angle_to_goal = std::atan2(dy, dx);

    // Get robot yaw
    tf2::Quaternion q(
        odo_.pose.pose.orientation.x,
        odo_.pose.pose.orientation.y,
        odo_.pose.pose.orientation.z,
        odo_.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double heading_diff = angle_to_goal - yaw;
    while (heading_diff > M_PI) heading_diff -= 2 * M_PI;
    while (heading_diff < -M_PI) heading_diff += 2 * M_PI;

    // Check if goal is within a forward cone and close enough
    if (dist_to_goal > max_distance || std::abs(heading_diff) > max_angle_rad)
        return false;

    // Check if any laser point is *between* robot and goal, blocking it
    for (const auto &obs : getLaserPoints())
    {
        double obs_dx = obs.x - robot_pos.x;
        double obs_dy = obs.y - robot_pos.y;
        double obs_dist = std::hypot(obs_dx, obs_dy);

        // Skip if obstacle is farther than the goal
        if (obs_dist >= dist_to_goal) continue;

        // Compute lateral deviation from the line to the goal
        double proj = (obs_dx * dx + obs_dy * dy) / dist_to_goal;
        double lateral = std::sqrt(std::pow(obs_dx, 2) + std::pow(obs_dy, 2) - std::pow(proj, 2));

        if (lateral < clearance_radius)
            return false;  // Something is blocking directly in front
    }

    return true;  // No blocking obstacle before goal
}





geometry_msgs::msg::Point ObstacleAvoidance::getFurthestFrontLaserPoint(
    const geometry_msgs::msg::Point &robot_pos,
    const std::vector<geometry_msgs::msg::Point> &laser_points)
{
    double max_dist = -1.0;
    geometry_msgs::msg::Point best_point = robot_pos;  // fallback

    for (const auto &pt : laser_points) {
        double dx = pt.x - robot_pos.x;
        double dy = pt.y - robot_pos.y;
        double angle = std::atan2(dy, dx);
        double dist = std::hypot(dx, dy);

        if (std::abs(angle) <= M_PI / 2.0 && dist > max_dist) {  // within ±90°
            max_dist = dist;
            best_point = pt;
        }
    }
    return best_point;
}









//////////////////////////////////////////////////

////with extra repulsion
// geometry_msgs::msg::PoseStamped ObstacleAvoidance::suggestNewGoal(
//     const geometry_msgs::msg::PoseStamped &original_goal,
//     const geometry_msgs::msg::PoseStamped &next_goal,
//     double attraction,
//     double repulsion,
//     double goal_step)
// {
//     geometry_msgs::msg::Point robot_pos = odo_.pose.pose.position;
//     std::vector<geometry_msgs::msg::Point> obstacles = getLaserPoints();

//     const double Q_attraction = attraction;
//     const double Q_repulsion = repulsion;
//     const double GOAL_STEP = goal_step;

//     const double INFLUENCE_RADIUS = 0.325;  // distance within which obstacles repel
//     const double SAFE_DISTANCE = 0.35;     // min allowed distance from obstacles
//     const double ADJUST_STEP = 0.05;      // how much to nudge goal per adjustment
//     const int MAX_ITERATIONS = 20;        // avoid infinite loops

//     // --- Attraction Vector (towards original goal) ---
//     double dx = original_goal.pose.position.x - robot_pos.x;
//     double dy = original_goal.pose.position.y - robot_pos.y;
//     double dist = std::hypot(dx, dy);

//     double fx_att = 0.0, fy_att = 0.0;
//     if (dist > 1e-3) {
//         double F_att = Q_attraction / (4 * M_PI * dist * dist);
//         fx_att = F_att * dx / dist;
//         fy_att = F_att * dy / dist;
//     }

//     // --- Repulsion Vector (away from nearby obstacles) ---
//     double fx_rep = 0.0, fy_rep = 0.0;
//     for (const auto &obs : obstacles) {
//         double dx_o = robot_pos.x - obs.x;
//         double dy_o = robot_pos.y - obs.y;
//         double d_o = std::hypot(dx_o, dy_o);

//         // if (d_o < INFLUENCE_RADIUS && d_o > 1e-3) {
//         //     double scale = (1.0 / d_o - 1.0 / INFLUENCE_RADIUS);
//         //     double F_rep = Q_repulsion * scale / (d_o * d_o);
//         //     fx_rep += F_rep * dx_o / d_o;
//         //     fy_rep += F_rep * dy_o / d_o;
//         // }
//     }

//     // --- Resultant Force Vector ---
//     double fx = fx_att + fx_rep;
//     double fy = fy_att + fy_rep;
//     double norm = std::hypot(fx, fy);

//     geometry_msgs::msg::PoseStamped suggested;
//     suggested.header = original_goal.header;
//     suggested.pose.position.x = robot_pos.x + (norm > 1e-5 ? GOAL_STEP * fx / norm : 0.0);
//     suggested.pose.position.y = robot_pos.y + (norm > 1e-5 ? GOAL_STEP * fy / norm : 0.0);
//     suggested.pose.position.z = 0.0;
//     suggested.pose.orientation.w = 1.0;

//     // --- Safety Check: Move away from obstacles if too close ---
//     // bool safe = false;
//     // int count = 0;

//     // while (!safe && count++ < MAX_ITERATIONS) {
//     //     safe = true;
//     //     for (const auto &point : obstacles) {
//     //         double dx = suggested.pose.position.x - point.x;
//     //         double dy = suggested.pose.position.y - point.y;
//     //         double dist_to_obs = std::hypot(dx, dy);

//     //         if (dist_to_obs < SAFE_DISTANCE && dist_to_obs > 1e-3) {
//     //             // Nudge away from obstacle
//     //             double unit_dx = dx / dist_to_obs;
//     //             double unit_dy = dy / dist_to_obs;
//     //             suggested.pose.position.x += ADJUST_STEP * unit_dx;
//     //             suggested.pose.position.y += ADJUST_STEP * unit_dy;
//     //             safe = false;
//     //             break;  // recheck all obstacles after each nudge
//     //         }
//     //     }
//     // }

//     RCLCPP_WARN(rclcpp::get_logger("ObstacleAvoidance"), 
//                 "Suggesting new goal: (%.2f, %.2f)",
//                 suggested.pose.position.x, suggested.pose.position.y);

//     return suggested;
// }





void ObstacleAvoidance::odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
{
    odo_ = *msg;
}

nav_msgs::msg::Odometry ObstacleAvoidance::getOdometry(void)
{
    nav_msgs::msg::Odometry pose = odo_;
    return pose;
}

void ObstacleAvoidance::laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg)
{
    scan_ = *msg;
}

geometry_msgs::msg::Point ObstacleAvoidance::localToGlobal(nav_msgs::msg::Odometry global, geometry_msgs::msg::Point local) const
{
    geometry_msgs::msg::Point pt;

    pt.x = global.pose.pose.position.x + (local.x * cos(tf2::getYaw(global.pose.pose.orientation))) - (local.y * sin(tf2::getYaw(global.pose.pose.orientation)));
    pt.y = global.pose.pose.position.y + (local.x * sin(tf2::getYaw(global.pose.pose.orientation))) + (local.y * cos(tf2::getYaw(global.pose.pose.orientation)));
    pt.z = global.pose.pose.position.z;

    return pt;
}



bool ObstacleAvoidance::isPathObstructed()
{
    const double FRONT_ARC = M_PI / 4.0;  // ±60 degrees
    const double SAFE_DIST = 0.325;

    tf2::Quaternion q(
        odo_.pose.pose.orientation.x,
        odo_.pose.pose.orientation.y,
        odo_.pose.pose.orientation.z,
        odo_.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    geometry_msgs::msg::Point robot_pos = odo_.pose.pose.position;

    for (const auto &point : getLaserPoints())
    {
        double dx = point.x - robot_pos.x;
        double dy = point.y - robot_pos.y;
        double angle = std::atan2(dy, dx) - yaw;
        double dist = std::hypot(dx, dy);

        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;

        if (std::fabs(angle) < FRONT_ARC && dist < SAFE_DIST)
        {
            RCLCPP_WARN(this->get_logger(), "⚠️ Path obstructed: angle %.2f°, distance %.2f m", angle * 180.0 / M_PI, dist);
            return true;
        }
    }
    return false;
}


geometry_msgs::msg::Twist ObstacleAvoidance::avoidCollision()
{
    geometry_msgs::msg::Twist avoidance_cmd;
    avoidance_cmd.linear.x = 0.0;
    avoidance_cmd.angular.z = 0.3;

    RCLCPP_WARN(this->get_logger(), "⚠️ Obstacle ahead! Turning to avoid.");
    return avoidance_cmd;
}



// geometry_msgs::msg::PoseStamped ObstacleAvoidance::suggestNewGoalSafe()
// {
//     geometry_msgs::msg::Point robot_pos = odo_.pose.pose.position;
//     geometry_msgs::msg::Point target = getFurthestFrontLaserPoint(robot_pos, getLaserPoints());

//     geometry_msgs::msg::PoseStamped safe_goal;
//     safe_goal.header.frame_id = "odom";
//     safe_goal.header.stamp = this->get_clock()->now();
//     safe_goal.pose.position = target;

//     tf2::Quaternion q;
//     q.setRPY(0, 0, 0);
//     safe_goal.pose.orientation = tf2::toMsg(q);

//     return safe_goal;
// }

geometry_msgs::msg::PoseStamped ObstacleAvoidance::suggestNewGoalSafe()
{
    geometry_msgs::msg::Point robot_pos = odo_.pose.pose.position;
    geometry_msgs::msg::Point target = getFurthestFrontLaserPoint(robot_pos, getLaserPoints());

    // Step 1: compute direction vector
    double dx = target.x - robot_pos.x;
    double dy = target.y - robot_pos.y;
    double dist = std::hypot(dx, dy);

    // Step 2: normalize and apply fixed step
    const double STEP_SIZE = 0.25;  // limit how far the robot escapes

    geometry_msgs::msg::Point safe_point;
    if (dist > 1e-3)
    {
        safe_point.x = robot_pos.x + STEP_SIZE * dx / dist;
        safe_point.y = robot_pos.y + STEP_SIZE * dy / dist;
    }
    else
    {
        safe_point = robot_pos;  // fallback (no clear direction)
    }

    // Step 3: create PoseStamped goal
    geometry_msgs::msg::PoseStamped safe_goal;
    safe_goal.header.frame_id = "odom";
    safe_goal.header.stamp = this->get_clock()->now();
    safe_goal.pose.position = safe_point;

    // Face in direction of motion
    double yaw = std::atan2(dy, dx);
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    safe_goal.pose.orientation = tf2::toMsg(q);

    RCLCPP_INFO(this->get_logger(), "Suggesting minimal escape goal (%.2f, %.2f)", safe_point.x, safe_point.y);
    return safe_goal;
}

