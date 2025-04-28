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

bool ObstacleAvoidance::isGoalObstructed(const nav_msgs::msg::Odometry &odom, const geometry_msgs::msg::Point &goal)
{
    std::vector<geometry_msgs::msg::Point> points = getLaserPoints();

    // Get robot's orientation
    tf2::Quaternion q(
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);  // yaw = robot’s heading

    const double FRONT_ANGLE_CONE = M_PI; // 90 degrees total (±45°)
    const double COLLISION_DISTANCE = 0.2;    // danger zone

    for (const auto &point : points)
    {
        double dx = point.x - odom.pose.pose.position.x;
        double dy = point.y - odom.pose.pose.position.y;
        double distance = std::hypot(dx, dy);

        if (distance > COLLISION_DISTANCE)
            continue;

        double angle_to_point = std::atan2(dy, dx);
        double angle_diff = angle_to_point - yaw;

        // Normalize to [-π, π]
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        // Check if within frontal cone
        if (std::fabs(angle_diff) < FRONT_ANGLE_CONE / 2)
        {
            RCLCPP_WARN(rclcpp::get_logger("ObstacleAvoidance"), 
                        "collision avoidance kicking in, Angle: %.1f°, Distance: %.2f m",
                        angle_diff * 180.0 / M_PI, distance);
            return true;
        }
    }

    return false;
}


bool ObstacleAvoidance::isGoalInsideObstacle(const nav_msgs::msg::Odometry &odom, const geometry_msgs::msg::Point &goal) 
{
    std::vector<geometry_msgs::msg::Point> points = getLaserPoints();

    const double DISTANCE_THRESHOLD = 0.2;           // Max distance between laser point and goal
    const double ANGLE_THRESHOLD = M_PI / 6.0;       // ±30 degrees cone

    geometry_msgs::msg::Point robot_pos = odom.pose.pose.position;

    // Direction to the goal from the robot
    double dx_goal = goal.x - robot_pos.x;
    double dy_goal = goal.y - robot_pos.y;
    double goal_distance = std::hypot(dx_goal, dy_goal);
    double angle_to_goal = std::atan2(dy_goal, dx_goal);

    for (const auto &point : points)
    {
        // Vector from robot to current point
        double dx = point.x - robot_pos.x;
        double dy = point.y - robot_pos.y;
        double distance_to_point = std::hypot(dx, dy);
        double angle_to_point = std::atan2(dy, dx);

        // Step 1: Check if the point is within angular range of the goal direction
        double angle_diff = angle_to_point - angle_to_goal;
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        if (std::fabs(angle_diff) > ANGLE_THRESHOLD)
            continue;  // Ignore points outside the angular range

        // Step 2: Check if the point lies in front of the goal (closer to robot than the goal is)
        if (distance_to_point > goal_distance + 0.2)
            continue;

        // Step 3: Check if the point is close enough to the goal
        double distance_to_goal = std::hypot(point.x - goal.x, point.y - goal.y);
        if (distance_to_goal < DISTANCE_THRESHOLD)
        {
            RCLCPP_WARN(rclcpp::get_logger("ObstacleAvoidance"),
                        "Goal at (%.2f, %.2f) is inside an obstacle!",
                        goal.x, goal.y);
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
//     const geometry_msgs::msg::PoseStamped &next_goal)
// {
//     geometry_msgs::msg::Point robot_pos = odo_.pose.pose.position;
//     std::vector<geometry_msgs::msg::Point> obstacles = getLaserPoints();

//     const double ATTRACTIVE_GAIN = 1.2;
//     const double REPULSIVE_GAIN = 0.5;
//     const double INFLUENCE_RADIUS = 1.0;
//     const double GOAL_STEP = 0.5;

//     // --- 1. Attractive Force (towards original goal) ---
//     double fx = ATTRACTIVE_GAIN * (next_goal.pose.position.x - robot_pos.x);
//     double fy = ATTRACTIVE_GAIN * (next_goal.pose.position.y - robot_pos.y);

//     // --- 2. Repulsive Forces (away from obstacles) ---
//     for (const auto &obs : obstacles)
//     {
//         double dx = robot_pos.x - obs.x;
//         double dy = robot_pos.y - obs.y;
//         double dist = std::hypot(dx, dy);

//         if (dist < INFLUENCE_RADIUS && dist > 0.05)
//         {
//             double repulsion = REPULSIVE_GAIN * (1.0 / dist - 1.0 / INFLUENCE_RADIUS) / (dist * dist);
//             fx += repulsion * (dx / dist);
//             fy += repulsion * (dy / dist);
//         }
//     }

//     // --- 3. Compute new position ---
//     double magnitude = std::hypot(fx, fy);
//     if (magnitude == 0)
//     {
//         return original_goal;
//     }

//     geometry_msgs::msg::PoseStamped new_goal;
//     new_goal.pose.position.x = robot_pos.x + GOAL_STEP * (fx / magnitude);
//     new_goal.pose.position.y = robot_pos.y + GOAL_STEP * (fy / magnitude);
//     new_goal.pose.position.z = 0;

//     // --- 4. Set orientation to face next_goal ---
//     double dx_orient = next_goal.pose.position.x - new_goal.pose.position.x;
//     double dy_orient = next_goal.pose.position.y - new_goal.pose.position.y;
//     double yaw = std::atan2(dy_orient, dx_orient);

//     tf2::Quaternion q;
//     q.setRPY(0, 0, yaw);
//     new_goal.pose.orientation = tf2::toMsg(q);

//     // RCLCPP_WARN(rclcpp::get_logger("ObstacleAvoidance"),
//     //             "Suggesting new goal: (%.2f, %.2f) facing toward (%.2f, %.2f)",
//     //             new_goal.pose.position.x, new_goal.pose.position.y,
//     //             next_goal.pose.position.x, next_goal.pose.position.y);

//     return new_goal;
// }


// geometry_msgs::msg::PoseStamped ObstacleAvoidance::suggestNewGoal(
//     const geometry_msgs::msg::PoseStamped &original_goal,
//     const geometry_msgs::msg::PoseStamped &next_goal)
// {
//     geometry_msgs::msg::Point robot_pos = odo_.pose.pose.position;
//     std::vector<geometry_msgs::msg::Point> obstacles = getLaserPoints();

//     const double ATTRACTIVE_GAIN = 1.0;
//     const double REPULSIVE_GAIN = 0.4;
//     const double INFLUENCE_RADIUS = 0.8;
//     const double GOAL_STEP = 0.2;
//     const double MAX_LATERAL_DEVIATION = 0.6;

//     // --- 1. Attractive Force towards path direction (robot → next_goal) ---
//     double fx = ATTRACTIVE_GAIN * (next_goal.pose.position.x - robot_pos.x);
//     double fy = ATTRACTIVE_GAIN * (next_goal.pose.position.y - robot_pos.y);

//     // --- 2. Repulsive Forces from nearby obstacles ---
//     for (const auto &obs : obstacles)
//     {
//         double dx = robot_pos.x - obs.x;
//         double dy = robot_pos.y - obs.y;
//         double dist = std::hypot(dx, dy);

//         if (dist < INFLUENCE_RADIUS && dist > 0.05)
//         {
//             double repulsion = REPULSIVE_GAIN * (1.0 / dist - 1.0 / INFLUENCE_RADIUS) / (dist * dist);
//             fx += repulsion * (dx / dist);
//             fy += repulsion * (dy / dist);
//         }
//     }

//     // --- 3. Normalize and step forward ---
//     double magnitude = std::hypot(fx, fy);
//     if (magnitude == 0)
//         return original_goal;

//     // Base new goal on direction vector
//     double new_x = robot_pos.x + GOAL_STEP * (fx / magnitude);
//     double new_y = robot_pos.y + GOAL_STEP * (fy / magnitude);

//     // --- 4. Clamp lateral deviation from path line ---
//     // Line: robot_pos → next_goal
//     double dx_path = next_goal.pose.position.x - robot_pos.x;
//     double dy_path = next_goal.pose.position.y - robot_pos.y;
//     double path_length = std::hypot(dx_path, dy_path);

//     if (path_length > 1e-3)
//     {
//         double nx = dx_path / path_length;
//         double ny = dy_path / path_length;

//         // Vector from robot to new proposed point
//         double dx_new = new_x - robot_pos.x;
//         double dy_new = new_y - robot_pos.y;

//         // Project onto path direction
//         double proj = dx_new * nx + dy_new * ny;

//         // Orthogonal deviation
//         double ortho_x = dx_new - proj * nx;
//         double ortho_y = dy_new - proj * ny;

//         double ortho_dist = std::hypot(ortho_x, ortho_y);

//         if (ortho_dist > MAX_LATERAL_DEVIATION)
//         {
//             // Clamp orthogonal deviation
//             ortho_x *= MAX_LATERAL_DEVIATION / ortho_dist;
//             ortho_y *= MAX_LATERAL_DEVIATION / ortho_dist;

//             new_x = robot_pos.x + proj * nx + ortho_x;
//             new_y = robot_pos.y + proj * ny + ortho_y;
//         }
//     }

//     // --- 5. Safety check: avoid suggesting a point inside an obstacle ---
//     geometry_msgs::msg::Point temp_point;
//     temp_point.x = new_x;
//     temp_point.y = new_y;
//     temp_point.z = 0;

//     if (isGoalInsideObstacle(odo_, temp_point))
//     {
//         RCLCPP_WARN(rclcpp::get_logger("ObstacleAvoidance"), "Suggested goal inside obstacle, fallback to current.");
//         return original_goal;
//     }

//     geometry_msgs::msg::PoseStamped new_goal;
//     new_goal.pose.position = temp_point;

//     // --- 6. Orientation towards next goal ---
//     double yaw = std::atan2(next_goal.pose.position.y - new_y, next_goal.pose.position.x - new_x);
//     tf2::Quaternion q;
//     q.setRPY(0, 0, yaw);
//     new_goal.pose.orientation = tf2::toMsg(q);

//     return new_goal;
// }

geometry_msgs::msg::PoseStamped ObstacleAvoidance::suggestNewGoal(
    const geometry_msgs::msg::PoseStamped &original_goal,
    const geometry_msgs::msg::PoseStamped &next_goal)
{
    geometry_msgs::msg::Point robot_pos = odo_.pose.pose.position;
    std::vector<geometry_msgs::msg::Point> obstacles = getLaserPoints();

    const double Q_attraction = 100;
    const double Q_repulsion = 15;
    const double MIN_DISTANCE = 0.1;
    const double MAX_DISTANCE = 100.0;
    const double GOAL_STEP = 0.5;

    // --- Attraction Vector ---
    double dx = original_goal.pose.position.x - robot_pos.x;
    double dy = original_goal.pose.position.y - robot_pos.y;
    double dist = std::hypot(dx, dy);

    double fx_att = 0.0;
    double fy_att = 0.0;
    if (dist > 1e-3) {
        double F_att = Q_attraction / (4 * M_PI * dist * dist);
        fx_att = F_att * dx / dist;
        fy_att = F_att * dy / dist;
    }

    // --- Repulsion Vector ---
    double fx_rep = 0.0;
    double fy_rep = 0.0;
    for (const auto &obs : obstacles) {
        double dx_o = robot_pos.x - obs.x;
        double dy_o = robot_pos.y - obs.y;
        double d_o = std::hypot(dx_o, dy_o);
        if (d_o < MIN_DISTANCE || d_o > MAX_DISTANCE) continue;

        double F_rep = Q_repulsion / (4 * M_PI * d_o * d_o);
        fx_rep += -F_rep * dx_o / d_o;
        fy_rep += -F_rep * dy_o / d_o;
    }

    // --- Final vector ---
    double fx = fx_att + fx_rep;
    double fy = fy_att + fy_rep;
    double norm = std::hypot(fx, fy);

    geometry_msgs::msg::PoseStamped suggested;
    suggested.header = original_goal.header;
    suggested.pose.position.x = robot_pos.x + (norm > 1e-5 ? GOAL_STEP * fx / norm : 0.0);
    suggested.pose.position.y = robot_pos.y + (norm > 1e-5 ? GOAL_STEP * fy / norm : 0.0);
    suggested.pose.position.z = 0.0;
    suggested.pose.orientation.w = 1.0;

    return suggested;
}




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
