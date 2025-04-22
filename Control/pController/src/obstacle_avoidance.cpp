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

    const double FRONT_ANGLE_CONE = M_PI / 2; // 90 degrees total (±45°)
    const double COLLISION_DISTANCE = 0.3;    // danger zone

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
                        "obstacle detected, cannot reach goal...calculating new path, Angle: %.1f°, Distance: %.2f m",
                        angle_diff * 180.0 / M_PI, distance);
            return true;
        }
    }

    return false;
}




geometry_msgs::msg::Twist ObstacleAvoidance::adjustVelocity(const geometry_msgs::msg::Twist &original_cmd)
{
    return original_cmd;
}

// geometry_msgs::msg::Point ObstacleAvoidance::suggestNewGoal(const geometry_msgs::msg::Point &original_goal)
// {
//     return original_goal;
// }

geometry_msgs::msg::PoseStamped ObstacleAvoidance::suggestNewGoal(
    const geometry_msgs::msg::PoseStamped &original_goal,
    const geometry_msgs::msg::PoseStamped &next_goal)
{
    geometry_msgs::msg::Point robot_pos = odo_.pose.pose.position;
    std::vector<geometry_msgs::msg::Point> obstacles = getLaserPoints();

    const double ATTRACTIVE_GAIN = 1.0;
    const double REPULSIVE_GAIN = 0.8;
    const double INFLUENCE_RADIUS = 1.0;
    const double GOAL_STEP = 0.5;

    // --- 1. Attractive Force (towards original goal) ---
    double fx = ATTRACTIVE_GAIN * (original_goal.pose.position.x - robot_pos.x);
    double fy = ATTRACTIVE_GAIN * (original_goal.pose.position.y - robot_pos.y);

    // --- 2. Repulsive Forces (away from obstacles) ---
    for (const auto &obs : obstacles)
    {
        double dx = robot_pos.x - obs.x;
        double dy = robot_pos.y - obs.y;
        double dist = std::hypot(dx, dy);

        if (dist < INFLUENCE_RADIUS && dist > 0.05)
        {
            double repulsion = REPULSIVE_GAIN * (1.0 / dist - 1.0 / INFLUENCE_RADIUS) / (dist * dist);
            fx += repulsion * (dx / dist);
            fy += repulsion * (dy / dist);
        }
    }

    // --- 3. Compute new position ---
    double magnitude = std::hypot(fx, fy);
    if (magnitude == 0)
    {
        return original_goal;
    }

    geometry_msgs::msg::PoseStamped new_goal;
    new_goal.pose.position.x = robot_pos.x + GOAL_STEP * (fx / magnitude);
    new_goal.pose.position.y = robot_pos.y + GOAL_STEP * (fy / magnitude);
    new_goal.pose.position.z = 0;

    // --- 4. Set orientation to face next_goal ---
    double dx_orient = next_goal.pose.position.x - new_goal.pose.position.x;
    double dy_orient = next_goal.pose.position.y - new_goal.pose.position.y;
    double yaw = std::atan2(dy_orient, dx_orient);

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    new_goal.pose.orientation = tf2::toMsg(q);

    RCLCPP_WARN(rclcpp::get_logger("ObstacleAvoidance"),
                "Suggesting new goal: (%.2f, %.2f) facing toward (%.2f, %.2f)",
                new_goal.pose.position.x, new_goal.pose.position.y,
                next_goal.pose.position.x, next_goal.pose.position.y);

    return new_goal;
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
