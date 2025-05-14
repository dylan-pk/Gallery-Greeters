#include "obstacle_avoidance.hpp"
#include <cmath>

ObstacleAvoidance::ObstacleAvoidance() : Node("obstacle_avoidance")
{
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&ObstacleAvoidance::laserCallback, this, std::placeholders::_1));

    static_obs_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/known_static_obstacles", 10, std::bind(&ObstacleAvoidance::staticObstaclesCallback, this, std::placeholders::_1));

    static_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/static_occupancy_grid", 1, std::bind(&ObstacleAvoidance::staticMapCallback, this, std::placeholders::_1));

    unknown_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/updated_occupancy_grid", 10);

    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ObstacleAvoidance::publishUnknownObstacles, this));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&ObstacleAvoidance::odomCallback, this, std::placeholders::_1));

    static_grid_client_ = this->create_client<std_srvs::srv::Trigger>("/request_static_grid");

    request_timer_ = this->create_wall_timer(
    std::chrono::seconds(2),
    [this]()
    {
        this->requestStaticGridOnce();
        this->request_timer_->cancel();  // one-shot
    });




}

void ObstacleAvoidance::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_odom_ = *msg;
}


void ObstacleAvoidance::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    latest_scan_ = *msg;
}

void ObstacleAvoidance::staticObstaclesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    known_static_obstacles_.clear();
    for (const auto &pose : msg->poses)
        known_static_obstacles_.push_back(pose.position);

    RCLCPP_INFO(this->get_logger(), "Updated static obstacle list: %zu entries", known_static_obstacles_.size());
}

void ObstacleAvoidance::requestStaticGridOnce()
{
    // Wait for service to be available
    if (!static_grid_client_->wait_for_service(std::chrono::seconds(2)))
    {
        RCLCPP_WARN(this->get_logger(), "⚠️ Service /request_static_grid not available.");
        return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    // Send the request asynchronously and bind a callback
    static_grid_client_->async_send_request(
        request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result)
        {
            auto response = result.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "✅ Static grid requested: %s", response->message.c_str());
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "❌ Static grid service failed: %s", response->message.c_str());
            }
        });
}




void ObstacleAvoidance::staticMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_info_ = msg->info;
    map_ready_ = true;
    // RCLCPP_INFO(this->get_logger(), "Received map info: %d x %d", map_info_.width, map_info_.height);
}

bool ObstacleAvoidance::is_known_static_obstacle(const geometry_msgs::msg::Point &pt, double threshold) const
{
    for (const auto &known : known_static_obstacles_)
        if (std::hypot(known.x - pt.x, known.y - pt.y) < threshold)
            return true;
    return false;
}

std::vector<geometry_msgs::msg::Point> ObstacleAvoidance::getFilteredLaserPoints() const
{
    std::vector<geometry_msgs::msg::Point> unknown_obstacles;

    if (latest_scan_.ranges.empty())
        return unknown_obstacles;

    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform("map", latest_scan_.header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF2 Error: %s", ex.what());
        return unknown_obstacles;
    }

    for (size_t i = 0; i < latest_scan_.ranges.size(); ++i)
    {
        float r = latest_scan_.ranges[i];
        if (!std::isfinite(r) || r < latest_scan_.range_min || r > latest_scan_.range_max)
            continue;

        float angle = latest_scan_.angle_min + i * latest_scan_.angle_increment;

        geometry_msgs::msg::PointStamped pt_local, pt_global;
        pt_local.header = latest_scan_.header;
        pt_local.point.x = r * std::cos(angle);
        pt_local.point.y = r * std::sin(angle);

        try {
            tf2::doTransform(pt_local, pt_global, tf);
            if (!is_known_static_obstacle(pt_global.point, 1.0))
                unknown_obstacles.push_back(pt_global.point);
        } catch (const tf2::TransformException &ex) {
            continue;
        }
    }

    return unknown_obstacles;
}

// void ObstacleAvoidance::publishUnknownObstacles()
// {
//     if (!map_ready_) return;

//     auto points = getFilteredLaserPoints();

//     nav_msgs::msg::OccupancyGrid grid;
//     grid.header.stamp = this->now();
//     grid.header.frame_id = "map";
//     grid.info = map_info_;
//     grid.data.resize(grid.info.width * grid.info.height, -1); // unknown

//     for (const auto &pt : points)
//     {
//         int gx = static_cast<int>((pt.x - grid.info.origin.position.x) / grid.info.resolution);
//         int gy = static_cast<int>((pt.y - grid.info.origin.position.y) / grid.info.resolution);

//         if (gx >= 0 && gx < static_cast<int>(grid.info.width) &&
//             gy >= 0 && gy < static_cast<int>(grid.info.height))
//         {
//             int flipped_gy = grid.info.height - 1 - gy;
//             int index = flipped_gy * grid.info.width + gx;
//             grid.data[index] = 100;
//         }
//     }

//     unknown_grid_pub_->publish(grid);
//     RCLCPP_INFO(this->get_logger(), "✅ Published updated occupancy grid with %zu unknown points", points.size());
// }


// void ObstacleAvoidance::publishUnknownObstacles()
// {
//     if (!map_ready_ || latest_scan_.ranges.empty()) return;

//     // Get the robot's position in the map frame
//     geometry_msgs::msg::TransformStamped tf;
//     try {
//         tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
//     } catch (const tf2::TransformException &ex) {
//         RCLCPP_WARN(this->get_logger(), "TF2 Error (robot pose): %s", ex.what());
//         return;
//     }

//     float robot_x = tf.transform.translation.x;
//     float robot_y = tf.transform.translation.y;

//     auto points = getFilteredLaserPoints();

//     // Filter: only keep unknown points within 0.5m of the robot
//     std::vector<geometry_msgs::msg::Point> nearby_points;
//     for (const auto &pt : points)
//     {
//         float dx = pt.x - robot_x;
//         float dy = pt.y - robot_y;
//         if (std::hypot(dx, dy) <= 0.5)
//             nearby_points.push_back(pt);
//     }

//     if (nearby_points.empty()) {
//         RCLCPP_INFO(this->get_logger(), "No nearby unknown obstacles — skipping grid publish.");
//         return;
//     }

//     // Create occupancy grid message
//     nav_msgs::msg::OccupancyGrid grid;
//     grid.header.stamp = this->now();
//     grid.header.frame_id = "map";
//     grid.info = map_info_;
//     grid.data.resize(grid.info.width * grid.info.height, -1); // All unknown

//     for (const auto &pt : nearby_points)
//     {
//         int gx = static_cast<int>((pt.x - grid.info.origin.position.x) / grid.info.resolution);
//         int gy = static_cast<int>((pt.y - grid.info.origin.position.y) / grid.info.resolution);

//         // Flip vertically to match ROS map orientation (if needed)
//         int flipped_gy = grid.info.height - 1 - gy;

//         if (gx >= 0 && gx < static_cast<int>(grid.info.width) &&
//             flipped_gy >= 0 && flipped_gy < static_cast<int>(grid.info.height))
//         {
//             int index = flipped_gy * grid.info.width + gx;
//             grid.data[index] = 100;
//         }
//     }

//     unknown_grid_pub_->publish(grid);
//     RCLCPP_INFO(this->get_logger(), "✅ Published %zu unknown obstacle(s) within 0.5m", nearby_points.size());
// }
void ObstacleAvoidance::publishUnknownObstacles()
{
    if (!map_ready_ || latest_scan_.ranges.empty())
        return;

    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform("map", latest_scan_.header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF2 Error: %s", ex.what());
        return;
    }

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = this->now();
    grid.header.frame_id = "map";
    grid.info = map_info_;
    grid.data.resize(grid.info.width * grid.info.height, -1); // initialize all as unknown

    bool has_segment = false;

    for (size_t i = 1; i < latest_scan_.ranges.size(); ++i)
    {
        float r1 = latest_scan_.ranges[i - 1];
        float r2 = latest_scan_.ranges[i];

        if (!std::isfinite(r1) || !std::isfinite(r2))
            continue;

        float angle1 = latest_scan_.angle_min + (i - 1) * latest_scan_.angle_increment;
        float angle2 = latest_scan_.angle_min + i * latest_scan_.angle_increment;

        geometry_msgs::msg::PointStamped pt1, pt2, global1, global2;
        pt1.header = pt2.header = latest_scan_.header;
        pt1.point.x = r1 * std::cos(angle1);
        pt1.point.y = r1 * std::sin(angle1);
        pt2.point.x = r2 * std::cos(angle2);
        pt2.point.y = r2 * std::sin(angle2);

        try {
            tf2::doTransform(pt1, global1, tf);
            tf2::doTransform(pt2, global2, tf);
        } catch (...) {
            continue;
        }

        // Validate segment (points are close to each other)
        float segment_length = std::hypot(global2.point.x - global1.point.x, global2.point.y - global1.point.y);
        if (segment_length > 0.15)
            continue;

        // Check distance from robot to midpoint
        float mid_x = (global1.point.x + global2.point.x) / 2.0;
        float mid_y = (global1.point.y + global2.point.y) / 2.0;
        float dist_to_robot = std::hypot(mid_x - current_odom_.pose.pose.position.x,
                                         mid_y - current_odom_.pose.pose.position.y);
        if (dist_to_robot > 0.5)
            continue;

        // Reject segment if midpoint is near a known static obstacle
        geometry_msgs::msg::Point midpoint;
        midpoint.x = mid_x;
        midpoint.y = mid_y;
        midpoint.z = 0.0;

        if (is_known_static_obstacle(midpoint, 0.25))  // Adjust threshold here
            continue;

        has_segment = true;

        // Mark interpolated segment points in grid
        for (float t = 0.0f; t <= 1.0f; t += 0.05f)

        {
            float x = global1.point.x * (1 - t) + global2.point.x * t;
            float y = global1.point.y * (1 - t) + global2.point.y * t;

            int gx = static_cast<int>((x - grid.info.origin.position.x) / grid.info.resolution);
            int gy = static_cast<int>((y - grid.info.origin.position.y) / grid.info.resolution);

            // === Flip y-axis to match ROS map convention ===
            int flipped_gy = grid.info.height - 1 - gy;

            if (gx >= 0 && gx < static_cast<int>(grid.info.width) &&
                flipped_gy >= 0 && flipped_gy < static_cast<int>(grid.info.height))
            {
                int idx = flipped_gy * grid.info.width + gx;
                grid.data[idx] = 100;
            }
        }
    }

    if (has_segment) {
        // unknown_grid_pub_->publish(grid); //commented out to disable publishing grid for testing
        // RCLCPP_INFO(this->get_logger(), "✅ Published segment-based unknown obstacle map.");
    }
}
