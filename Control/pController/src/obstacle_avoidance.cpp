// // #include "obstacle_avoidance.hpp"
// // #include <cmath>

// // ObstacleAvoidance::ObstacleAvoidance() : Node("obstacle_avoidance")
// // {
// //     laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
// //         "/scan", 10, std::bind(&ObstacleAvoidance::laserCallback, this, std::placeholders::_1));

// //     static_obs_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
// //         "/known_static_obstacles", 10, std::bind(&ObstacleAvoidance::staticObstaclesCallback, this, std::placeholders::_1));

// //     static_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
// //         "/static_occupancy_grid", 1, std::bind(&ObstacleAvoidance::staticMapCallback, this, std::placeholders::_1));

// //     unknown_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/updated_occupancy_grid", 10);

// //     timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ObstacleAvoidance::publishUnknownObstacles, this));

// //     tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
// //     tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

// // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
// //     "/odom", 10,
// //     std::bind(&ObstacleAvoidance::odomCallback, this, std::placeholders::_1));

// //     static_grid_client_ = this->create_client<std_srvs::srv::Trigger>("/request_static_grid");

// //     request_timer_ = this->create_wall_timer(
// //     std::chrono::seconds(2),
// //     [this]()
// //     {
// //         this->requestStaticGridOnce();
// //         this->request_timer_->cancel();  // one-shot
// //     });

// // visual_timer_ = this->create_wall_timer(
// //     std::chrono::milliseconds(1000),
// //     std::bind(&ObstacleAvoidance::displayStaticMap, this));

// // }

// // void ObstacleAvoidance::displayStaticMap()
// // {
// //     if (static_map_grid_.empty()) return;

// //     cv::Mat resized;
// //     cv::resize(static_map_grid_, resized, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST);
// //     cv::imshow("Static Occupancy Grid (ObstacleAvoidance)", resized);
// //     cv::waitKey(1);
// // }

// // void ObstacleAvoidance::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
// // {
// //     current_odom_ = *msg;
// // }

// // void ObstacleAvoidance::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
// // {
// //     latest_scan_ = *msg;
// // }

// // // void ObstacleAvoidance::staticObstaclesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
// // // {
// // //     known_static_obstacles_.clear();
// // //     for (const auto &pose : msg->poses)
// // //         known_static_obstacles_.push_back(pose.position);

// // //     RCLCPP_INFO(this->get_logger(), "Updated static obstacle list: %zu entries", known_static_obstacles_.size());
// // // }

// // // void ObstacleAvoidance::staticObstaclesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
// // // {
// // //     known_static_obstacles_.clear();
// // //     for (const auto &pose : msg->poses)
// // //         known_static_obstacles_.push_back(pose.position);

// // //     // Build FLANN matrix
// // //     known_obstacles_mat_ = cv::Mat(known_static_obstacles_.size(), 2, CV_32F);
// // //     for (size_t i = 0; i < known_static_obstacles_.size(); ++i) {
// // //         known_obstacles_mat_.at<float>(i, 0) = known_static_obstacles_[i].x;
// // //         known_obstacles_mat_.at<float>(i, 1) = known_static_obstacles_[i].y;
// // //     }

// // //     // Rebuild the KD-Tree
// // //     kdtree_ = std::make_unique<cv::flann::Index>(known_obstacles_mat_, cv::flann::KDTreeIndexParams(1));

// // //     RCLCPP_INFO(this->get_logger(), "Updated static obstacle list: %zu entries (KD-Tree built)", known_static_obstacles_.size());
// // // }

// // // void ObstacleAvoidance::staticObstaclesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
// // // {
// // //     known_static_obstacles_.clear();

// // //     cv::Mat data(msg->poses.size(), 2, CV_32F);
// // //     for (size_t i = 0; i < msg->poses.size(); ++i) {
// // //         float x = msg->poses[i].position.x;
// // //         float y = msg->poses[i].position.y;
// // //         data.at<float>(i, 0) = x;
// // //         data.at<float>(i, 1) = y;

// // //         // Store original for debugging or further use
// // //         geometry_msgs::msg::Point pt;
// // //         pt.x = x;
// // //         pt.y = y;
// // //         known_static_obstacles_.push_back(pt);
// // //     }

// // //     kdtree_ = std::make_unique<cv::flann::Index>(data, cv::flann::KDTreeIndexParams(1));

// // //     RCLCPP_INFO(this->get_logger(), "Updated known static obstacle KD-Tree with %zu points", known_static_obstacles_.size());
// // // }

// // void ObstacleAvoidance::staticObstaclesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
// // {
// //     known_static_obstacles_.clear();

// //     RCLCPP_INFO(this->get_logger(), "📬 Received PoseArray with %zu poses", msg->poses.size());

// //     if (msg->poses.empty()) {
// //         RCLCPP_WARN(this->get_logger(), "⚠️ Received empty PoseArray on /known_static_obstacles!");
// //         return;
// //     }

// //     cv::Mat data(msg->poses.size(), 2, CV_32F);

// //     for (size_t i = 0; i < msg->poses.size(); ++i)
// //     {
// //         float x = msg->poses[i].position.x;
// //         float y = msg->poses[i].position.y;

// //         data.at<float>(i, 0) = x;
// //         data.at<float>(i, 1) = y;

// //         geometry_msgs::msg::Point pt;
// //         pt.x = x;
// //         pt.y = y;
// //         known_static_obstacles_.push_back(pt);
// //     }

// //     kdtree_ = std::make_unique<cv::flann::Index>(data, cv::flann::KDTreeIndexParams(1));

// //     RCLCPP_INFO(this->get_logger(), "✅ Received and stored %zu static obstacles, KDTree built.", known_static_obstacles_.size());
// // }

// // void ObstacleAvoidance::requestStaticGridOnce()
// // {
// //     // Wait for service to be available
// //     if (!static_grid_client_->wait_for_service(std::chrono::seconds(2)))
// //     {
// //         RCLCPP_WARN(this->get_logger(), "⚠️ Service /request_static_grid not available.");
// //         return;
// //     }

// //     auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

// //     // Send the request asynchronously and bind a callback
// //     static_grid_client_->async_send_request(
// //         request,
// //         [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result)
// //         {
// //             auto response = result.get();
// //             if (response->success)
// //             {
// //                 RCLCPP_INFO(this->get_logger(), "✅ Static grid requested: %s", response->message.c_str());
// //             }
// //             else
// //             {
// //                 RCLCPP_WARN(this->get_logger(), "❌ Static grid service failed: %s", response->message.c_str());
// //             }
// //         });
// // }

// // void ObstacleAvoidance::staticMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
// // {
// //     map_info_ = msg->info;
// //     map_ready_ = true;

// //     static_map_grid_ = cv::Mat(map_info_.height, map_info_.width, CV_8UC1);

// //     for (int y = 0; y < map_info_.height; ++y)
// //     {
// //         for (int x = 0; x < map_info_.width; ++x)
// //         {
// //             int idx = y * map_info_.width + x;
// //             int val = msg->data[idx];

// //             // Map values to grayscale
// //             if (val == 100)
// //                 static_map_grid_.at<uchar>(y, x) = 0;     // Black: occupied
// //             else if (val == 0)
// //                 static_map_grid_.at<uchar>(y, x) = 255;   // White: free
// //             else
// //                 static_map_grid_.at<uchar>(y, x) = 127;   // Gray: unknown
// //         }
// //     }

// //     RCLCPP_INFO(this->get_logger(), "🗺️ Stored and converted occupancy grid.");
// //     RCLCPP_INFO(this->get_logger(), "🗺️ Received static map: %d x %d", map_info_.width, map_info_.height);

// // }

// // // bool ObstacleAvoidance::is_known_static_obstacle(const geometry_msgs::msg::Point &pt, double threshold) const
// // // {
// // //     for (const auto &known : known_static_obstacles_)
// // //         if (std::hypot(known.x - pt.x, known.y - pt.y) < threshold)
// // //             return true;
// // //     return false;
// // // }

// // bool ObstacleAvoidance::is_known_static_obstacle(const geometry_msgs::msg::Point &pt, double threshold) const
// // {
// //     if (!kdtree_ || known_static_obstacles_.empty()) {
// //         RCLCPP_WARN(this->get_logger(), "⚠️ KDTree or static obstacle list is empty");
// //         return false;
// //     }

// //     cv::Mat query = (cv::Mat_<float>(1, 2) << pt.x, pt.y);
// //     std::vector<int> indices(1);
// //     std::vector<float> dists(1);

// //     kdtree_->knnSearch(query, indices, dists, 1, cv::flann::SearchParams());

// //     RCLCPP_DEBUG(this->get_logger(), "Query point: (%.2f, %.2f) — Nearest dist: %.2f", pt.x, pt.y, std::sqrt(dists[0]));

// //     return std::sqrt(dists[0]) < threshold;
// // }

// // std::vector<geometry_msgs::msg::Point> ObstacleAvoidance::getFilteredLaserPoints() const
// // {
// //     std::vector<geometry_msgs::msg::Point> unknown_obstacles;

// //     if (latest_scan_.ranges.empty())
// //         return unknown_obstacles;

// //     if (!kdtree_ || known_static_obstacles_.empty()) {
// //         RCLCPP_WARN(this->get_logger(), "⚠️ Skipping LaserScan filtering: KDTree not built yet.");
// //         return {};
// //     }

// //     geometry_msgs::msg::TransformStamped tf;
// //     try {
// //         tf = tf_buffer_->lookupTransform("map", latest_scan_.header.frame_id, tf2::TimePointZero);
// //     } catch (const tf2::TransformException &ex) {
// //         RCLCPP_WARN(this->get_logger(), "TF2 Error: %s", ex.what());
// //         return unknown_obstacles;
// //     }

// //     for (size_t i = 0; i < latest_scan_.ranges.size(); ++i)
// //     {
// //         float r = latest_scan_.ranges[i];
// //         if (!std::isfinite(r) || r < latest_scan_.range_min || r > latest_scan_.range_max)
// //             continue;

// //         float angle = latest_scan_.angle_min + i * latest_scan_.angle_increment;

// //         geometry_msgs::msg::PointStamped pt_local, pt_global;
// //         pt_local.header = latest_scan_.header;
// //         pt_local.point.x = r * std::cos(angle);
// //         pt_local.point.y = r * std::sin(angle);

// //         try {
// //             tf2::doTransform(pt_local, pt_global, tf);
// //             if (!is_known_static_obstacle(pt_global.point, 0.35))
// //                 unknown_obstacles.push_back(pt_global.point);
// //             // RCLCPP_INFO(this->get_logger(), "Detected %zu unknown points from LaserScan", unknown_obstacles.size());
// //         } catch (const tf2::TransformException &ex) {
// //             continue;
// //         }
// //     }

// //     return unknown_obstacles;
// // }

// // // void ObstacleAvoidance::publishUnknownObstacles()
// // // {
// // //     if (!map_ready_) return;

// // //     auto points = getFilteredLaserPoints();

// // //     nav_msgs::msg::OccupancyGrid grid;
// // //     grid.header.stamp = this->now();
// // //     grid.header.frame_id = "map";
// // //     grid.info = map_info_;
// // //     grid.data.resize(grid.info.width * grid.info.height, -1); // unknown

// // //     for (const auto &pt : points)
// // //     {
// // //         int gx = static_cast<int>((pt.x - grid.info.origin.position.x) / grid.info.resolution);
// // //         int gy = static_cast<int>((pt.y - grid.info.origin.position.y) / grid.info.resolution);

// // //         if (gx >= 0 && gx < static_cast<int>(grid.info.width) &&
// // //             gy >= 0 && gy < static_cast<int>(grid.info.height))
// // //         {
// // //             int flipped_gy = grid.info.height - 1 - gy;
// // //             int index = flipped_gy * grid.info.width + gx;
// // //             grid.data[index] = 100;
// // //         }
// // //     }

// // //     unknown_grid_pub_->publish(grid);
// // //     RCLCPP_INFO(this->get_logger(), "✅ Published updated occupancy grid with %zu unknown points", points.size());
// // // }

// // // void ObstacleAvoidance::publishUnknownObstacles()
// // // {
// // //     if (!map_ready_ || latest_scan_.ranges.empty()) return;
// // // //
// // //     // Get the robot's position in the map frame
// // //     geometry_msgs::msg::TransformStamped tf;
// // //     try {
// // //         tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
// // //     } catch (const tf2::TransformException &ex) {
// // //         RCLCPP_WARN(this->get_logger(), "TF2 Error (robot pose): %s", ex.what());
// // //         return;
// // //     }
// // // //
// // //     float robot_x = tf.transform.translation.x;
// // //     float robot_y = tf.transform.translation.y;
// // // //
// // //     auto points = getFilteredLaserPoints();
// // // //
// // //     // Filter: only keep unknown points within 0.5m of the robot
// // //     std::vector<geometry_msgs::msg::Point> nearby_points;
// // //     for (const auto &pt : points)
// // //     {
// // //         float dx = pt.x - robot_x;
// // //         float dy = pt.y - robot_y;
// // //         if (std::hypot(dx, dy) <= 0.5)
// // //             nearby_points.push_back(pt);
// // //     }
// // // //
// // //     if (nearby_points.empty()) {
// // //         RCLCPP_INFO(this->get_logger(), "No nearby unknown obstacles — skipping grid publish.");
// // //         return;
// // //     }
// // // //
// // //     // Create occupancy grid message
// // //     nav_msgs::msg::OccupancyGrid grid;
// // //     grid.header.stamp = this->now();
// // //     grid.header.frame_id = "map";
// // //     grid.info = map_info_;
// // //     grid.data.resize(grid.info.width * grid.info.height, -1); // All unknown
// // // //
// // //     for (const auto &pt : nearby_points)
// // //     {
// // //         int gx = static_cast<int>((pt.x - grid.info.origin.position.x) / grid.info.resolution);
// // //         int gy = static_cast<int>((pt.y - grid.info.origin.position.y) / grid.info.resolution);
// // // //
// // //         // Flip vertically to match ROS map orientation (if needed)
// // //         int flipped_gy = grid.info.height - 1 - gy;
// // // //
// // //         if (gx >= 0 && gx < static_cast<int>(grid.info.width) &&
// // //             flipped_gy >= 0 && flipped_gy < static_cast<int>(grid.info.height))
// // //         {
// // //             int index = flipped_gy * grid.info.width + gx;
// // //             grid.data[index] = 100;
// // //         }
// // //     }
// // // //
// // //     unknown_grid_pub_->publish(grid);
// // //     RCLCPP_INFO(this->get_logger(), "✅ Published %zu unknown obstacle(s) within 0.5m", nearby_points.size());
// // // }

// // ///////////////above is normal below is with cluster points

// // void ObstacleAvoidance::publishUnknownObstacles()
// // {
// //     if (!map_ready_ || latest_scan_.ranges.empty()) return;

// //     // Get the robot's position in the map frame
// //     geometry_msgs::msg::TransformStamped tf;
// //     try {
// //         tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
// //     } catch (const tf2::TransformException &ex) {
// //         RCLCPP_WARN(this->get_logger(), "TF2 Error (robot pose): %s", ex.what());
// //         return;
// //     }

// //     float robot_x = tf.transform.translation.x;
// //     float robot_y = tf.transform.translation.y;

// //     // Step 1: Get raw unknown points from laser scan (excluding known static obstacles via KD-tree)
// //     auto raw_points = getFilteredLaserPoints();

// //     // Step 2: Cluster to reduce noise and redundancy
// //     auto clustered_points = clusterPoints(raw_points, 0.2 /* radius in meters */, 2 /* min cluster size */);

// //     // Step 3: Filter for proximity to robot
// //     std::vector<geometry_msgs::msg::Point> nearby_points;
// //     for (const auto &pt : clustered_points)
// //     {
// //         float dx = pt.x - robot_x;
// //         float dy = pt.y - robot_y;
// //         if (std::hypot(dx, dy) <= 1)  // Within 0.5m radius
// //             nearby_points.push_back(pt);
// //     }

// //     if (nearby_points.empty()) {
// //         RCLCPP_INFO(this->get_logger(), "No nearby unknown obstacles — skipping grid publish.");
// //         return;
// //     }

// //     // Step 4: Create occupancy grid with just the nearby unknown obstacle points
// //     nav_msgs::msg::OccupancyGrid grid;
// //     grid.header.stamp = this->now();
// //     grid.header.frame_id = "map";
// //     grid.info = map_info_;
// //     grid.data.resize(grid.info.width * grid.info.height, -1); // All unknown

// //     for (const auto &pt : nearby_points)
// //     {
// //         int gx = static_cast<int>((pt.x - grid.info.origin.position.x) / grid.info.resolution);
// //         int gy = static_cast<int>((pt.y - grid.info.origin.position.y) / grid.info.resolution);
// //         int flipped_gy = grid.info.height - 1 - gy;

// //         if (gx >= 0 && gx < static_cast<int>(grid.info.width) &&
// //             flipped_gy >= 0 && flipped_gy < static_cast<int>(grid.info.height))
// //         {
// //             int index = flipped_gy * grid.info.width + gx;
// //             grid.data[index] = 100;
// //         }
// //     }

// //     unknown_grid_pub_->publish(grid);
// //     RCLCPP_INFO(this->get_logger(), "✅ Published %zu clustered unknown obstacle(s) within 0.5m", nearby_points.size());
// // }

// // ///////////////////////////////idek whta is below
// // // void ObstacleAvoidance::publishUnknownObstacles()
// // // {
// // //     if (!map_ready_ || latest_scan_.ranges.empty())
// // //         return;

// // //     geometry_msgs::msg::TransformStamped tf;
// // //     try {
// // //         tf = tf_buffer_->lookupTransform("map", latest_scan_.header.frame_id, tf2::TimePointZero);
// // //     } catch (const tf2::TransformException &ex) {
// // //         RCLCPP_WARN(this->get_logger(), "TF2 Error: %s", ex.what());
// // //         return;
// // //     }

// // //     nav_msgs::msg::OccupancyGrid grid;
// // //     grid.header.stamp = this->now();
// // //     grid.header.frame_id = "map";
// // //     grid.info = map_info_;
// // //     grid.data.resize(grid.info.width * grid.info.height, -1); // initialize all as unknown

// // //     bool has_segment = false;

// // //     for (size_t i = 1; i < latest_scan_.ranges.size(); ++i)
// // //     {
// // //         float r1 = latest_scan_.ranges[i - 1];
// // //         float r2 = latest_scan_.ranges[i];

// // //         if (!std::isfinite(r1) || !std::isfinite(r2))
// // //             continue;

// // //         float angle1 = latest_scan_.angle_min + (i - 1) * latest_scan_.angle_increment;
// // //         float angle2 = latest_scan_.angle_min + i * latest_scan_.angle_increment;

// // //         geometry_msgs::msg::PointStamped pt1, pt2, global1, global2;
// // //         pt1.header = pt2.header = latest_scan_.header;
// // //         pt1.point.x = r1 * std::cos(angle1);
// // //         pt1.point.y = r1 * std::sin(angle1);
// // //         pt2.point.x = r2 * std::cos(angle2);
// // //         pt2.point.y = r2 * std::sin(angle2);

// // //         try {
// // //             tf2::doTransform(pt1, global1, tf);
// // //             tf2::doTransform(pt2, global2, tf);
// // //         } catch (...) {
// // //             continue;
// // //         }

// // //         // Validate segment (points are close to each other)
// // //         float segment_length = std::hypot(global2.point.x - global1.point.x, global2.point.y - global1.point.y);
// // //         if (segment_length > 0.15)
// // //             continue;

// // //         // Check distance from robot to midpoint
// // //         float mid_x = (global1.point.x + global2.point.x) / 2.0;
// // //         float mid_y = (global1.point.y + global2.point.y) / 2.0;
// // //         float dist_to_robot = std::hypot(mid_x - current_odom_.pose.pose.position.x,
// // //                                          mid_y - current_odom_.pose.pose.position.y);
// // //         if (dist_to_robot > 0.5)
// // //             continue;

// // //         // Reject segment if midpoint is near a known static obstacle
// // //         geometry_msgs::msg::Point midpoint;
// // //         midpoint.x = mid_x;
// // //         midpoint.y = mid_y;
// // //         midpoint.z = 0.0;

// // //         if (is_known_static_obstacle(midpoint, 0.25))  // Adjust threshold here
// // //             continue;

// // //         has_segment = true;

// // //         // Mark interpolated segment points in grid
// // //         for (float t = 0.0f; t <= 1.0f; t += 0.05f)

// // //         {
// // //             float x = global1.point.x * (1 - t) + global2.point.x * t;
// // //             float y = global1.point.y * (1 - t) + global2.point.y * t;

// // //             int gx = static_cast<int>((x - grid.info.origin.position.x) / grid.info.resolution);
// // //             int gy = static_cast<int>((y - grid.info.origin.position.y) / grid.info.resolution);

// // //             // === Flip y-axis to match ROS map convention ===
// // //             int flipped_gy = grid.info.height - 1 - gy;

// // //             if (gx >= 0 && gx < static_cast<int>(grid.info.width) &&
// // //                 flipped_gy >= 0 && flipped_gy < static_cast<int>(grid.info.height))
// // //             {
// // //                 int idx = flipped_gy * grid.info.width + gx;
// // //                 grid.data[idx] = 100;
// // //             }
// // //         }
// // //     }

// // //     if (has_segment) {
// // //         unknown_grid_pub_->publish(grid); //commented out to disable publishing grid for testing
// // //         // RCLCPP_INFO(this->get_logger(), "✅ Published segment-based unknown obstacle map.");
// // //     }
// // // }

// // std::vector<geometry_msgs::msg::Point> ObstacleAvoidance::clusterPoints(
// //     const std::vector<geometry_msgs::msg::Point> &points, double cluster_radius, int min_cluster_size)
// // {
// //     std::vector<geometry_msgs::msg::Point> clustered;

// //     std::vector<bool> visited(points.size(), false);

// //     for (size_t i = 0; i < points.size(); ++i)
// //     {
// //         if (visited[i])
// //             continue;

// //         std::vector<size_t> cluster_indices;
// //         cluster_indices.push_back(i);
// //         visited[i] = true;

// //         geometry_msgs::msg::Point centroid = points[i];
// //         int cluster_size = 1;

// //         for (size_t j = i + 1; j < points.size(); ++j)
// //         {
// //             if (visited[j])
// //                 continue;

// //             double dx = points[i].x - points[j].x;
// //             double dy = points[i].y - points[j].y;
// //             if (std::hypot(dx, dy) < cluster_radius)
// //             {
// //                 cluster_indices.push_back(j);
// //                 visited[j] = true;

// //                 centroid.x += points[j].x;
// //                 centroid.y += points[j].y;
// //                 cluster_size++;
// //             }
// //         }

// //         if (cluster_size >= min_cluster_size)
// //         {
// //             centroid.x /= cluster_size;
// //             centroid.y /= cluster_size;
// //             clustered.push_back(centroid);
// //         }
// //     }

// //     return clustered;
// // }

// #include "obstacle_avoidance.hpp"
// #include <cmath>

// ObstacleAvoidance::ObstacleAvoidance() : Node("obstacle_avoidance")
// {
//     laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//         "/scan", 10, std::bind(&ObstacleAvoidance::laserCallback, this, std::placeholders::_1));

//     static_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
//         "/static_occupancy_grid", 1, std::bind(&ObstacleAvoidance::staticMapCallback, this, std::placeholders::_1));

//     unknown_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/updated_occupancy_grid", 10);

//     timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ObstacleAvoidance::publishUnknownObstacles, this));

//     tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
//     tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//     odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//         "/odom", 10, std::bind(&ObstacleAvoidance::odomCallback, this, std::placeholders::_1));

//     static_grid_client_ = this->create_client<std_srvs::srv::Trigger>("/request_static_grid");

//     request_timer_ = this->create_wall_timer(
//         std::chrono::seconds(2),
//         [this]() {
//             this->requestStaticGridOnce();
//             this->request_timer_->cancel();
//         });

//     visual_timer_ = this->create_wall_timer(
//         std::chrono::milliseconds(1000),
//         std::bind(&ObstacleAvoidance::displayStaticMap, this));
// }

// void ObstacleAvoidance::staticMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
// {
//     map_info_ = msg->info;
//     map_ready_ = true;

//     // 1. Create raw grayscale map (for display and general use)
//     static_map_grid_ = cv::Mat(msg->info.height, msg->info.width, CV_8UC1);
//     for (int y = 0; y < msg->info.height; ++y)
//     {
//         for (int x = 0; x < msg->info.width; ++x)
//         {
//             int idx = y * msg->info.width + x;
//             int val = msg->data[idx];

//             if (val == 100)
//                 static_map_grid_.at<uchar>(y, x) = 255; // occupied
//             else if (val == 0)
//                 static_map_grid_.at<uchar>(y, x) = 0;   // free
//             else
//                 static_map_grid_.at<uchar>(y, x) = 127; // unknown
//         }
//     }

//     // 2. Apply dilation only to a copy used for KDTree building
//     cv::Mat dilated;
//     cv::dilate(static_map_grid_, dilated, cv::Mat(), cv::Point(-1, -1), 1); // 1 iteration of 3x3 kernel

//     // 3. Extract obstacle points from the dilated image (NOT the visual one)
//     known_static_obstacles_.clear();
//     for (int y = 0; y < msg->info.height; ++y)
//     {
//         for (int x = 0; x < msg->info.width; ++x)
//         {
//             if (dilated.at<uchar>(y, x) == 255)
//             {
//                 geometry_msgs::msg::Point pt;
//                 pt.x = x * map_info_.resolution + map_info_.origin.position.x;
//                 pt.y = y * map_info_.resolution + map_info_.origin.position.y;
//                 pt.z = 0.0;
//                 known_static_obstacles_.push_back(pt);
//             }
//         }
//     }

//     // 4. Build KDTree
//     if (!known_static_obstacles_.empty())
//     {
//         cv::Mat data(known_static_obstacles_.size(), 2, CV_32F);
//         for (size_t i = 0; i < known_static_obstacles_.size(); ++i)
//         {
//             data.at<float>(i, 0) = known_static_obstacles_[i].x;
//             data.at<float>(i, 1) = known_static_obstacles_[i].y;
//         }

//         kdtree_ = std::make_unique<cv::flann::Index>(data, cv::flann::KDTreeIndexParams(1));
//         RCLCPP_INFO(this->get_logger(), "✅ Built KDTree from dilated static occupancy grid (%zu points)", known_static_obstacles_.size());
//     }
//     else
//     {
//         RCLCPP_WARN(this->get_logger(), "⚠️ No static obstacles found after dilation.");
//     }

//     RCLCPP_INFO(this->get_logger(), "✅ Static occupancy grid received, visual map built, KDTree prepared.");
// }

// void ObstacleAvoidance::displayStaticMap()
// {
//     if (static_map_grid_.empty()) return;

//     cv::Mat resized;
//     cv::resize(static_map_grid_, resized, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST);
//     cv::imshow("Static Occupancy Grid (ObstacleAvoidance)", resized);
//     cv::waitKey(1);
// }

// void ObstacleAvoidance::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
// {
//     current_odom_ = *msg;
// }

// void ObstacleAvoidance::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
// {
//     latest_scan_ = *msg;
// }

// void ObstacleAvoidance::requestStaticGridOnce()
// {
//     if (!static_grid_client_->wait_for_service(std::chrono::seconds(2)))
//     {
//         RCLCPP_WARN(this->get_logger(), "⚠️ Service /request_static_grid not available.");
//         return;
//     }

//     auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

//     static_grid_client_->async_send_request(
//         request,
//         [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result)
//         {
//             auto response = result.get();
//             if (response->success)
//             {
//                 RCLCPP_INFO(this->get_logger(), "✅ Static grid requested: %s", response->message.c_str());
//             }
//             else
//             {
//                 RCLCPP_WARN(this->get_logger(), "❌ Static grid service failed: %s", response->message.c_str());
//             }
//         });
// }

// bool ObstacleAvoidance::is_known_static_obstacle(const geometry_msgs::msg::Point &pt, double threshold) const
// {
//     if (!kdtree_ || known_static_obstacles_.empty()) {
//         RCLCPP_WARN(this->get_logger(), "⚠️ KDTree or static obstacle list is empty");
//         return false;
//     }

//     cv::Mat query = (cv::Mat_<float>(1, 2) << pt.x, pt.y);
//     std::vector<int> indices(1);
//     std::vector<float> dists(1);

//     kdtree_->knnSearch(query, indices, dists, 1, cv::flann::SearchParams());

//     return std::sqrt(dists[0]) < threshold;
// }

// std::vector<geometry_msgs::msg::Point> ObstacleAvoidance::getFilteredLaserPoints() const
// {
//     std::vector<geometry_msgs::msg::Point> unknown_obstacles;

//     if (latest_scan_.ranges.empty())
//         return unknown_obstacles;

//     if (!kdtree_ || known_static_obstacles_.empty()) {
//         RCLCPP_WARN(this->get_logger(), "⚠️ Skipping LaserScan filtering: KDTree not built yet.");
//         return {};
//     }

//     geometry_msgs::msg::TransformStamped tf;
//     try {
//         tf = tf_buffer_->lookupTransform("map", latest_scan_.header.frame_id, tf2::TimePointZero);
//     } catch (const tf2::TransformException &ex) {
//         RCLCPP_WARN(this->get_logger(), "TF2 Error: %s", ex.what());
//         return unknown_obstacles;
//     }

//     for (size_t i = 0; i < latest_scan_.ranges.size(); ++i)
//     {
//         float r = latest_scan_.ranges[i];
//         if (!std::isfinite(r) || r < latest_scan_.range_min || r > latest_scan_.range_max)
//             continue;

//         float angle = latest_scan_.angle_min + i * latest_scan_.angle_increment;

//         geometry_msgs::msg::PointStamped pt_local, pt_global;
//         pt_local.header = latest_scan_.header;
//         pt_local.point.x = r * std::cos(angle);
//         pt_local.point.y = r * std::sin(angle);

//         try {
//             tf2::doTransform(pt_local, pt_global, tf);
//             if (!is_known_static_obstacle(pt_global.point, 0.25))
//                 unknown_obstacles.push_back(pt_global.point);
//         } catch (const tf2::TransformException &ex) {
//             continue;
//         }
//     }

//     return unknown_obstacles;
// }

// void ObstacleAvoidance::publishUnknownObstacles()
// {
//     if (!map_ready_ || latest_scan_.ranges.empty()) return;

//     geometry_msgs::msg::TransformStamped tf;
//     try {
//         tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
//     } catch (const tf2::TransformException &ex) {
//         RCLCPP_WARN(this->get_logger(), "TF2 Error (robot pose): %s", ex.what());
//         return;
//     }

//     float robot_x = tf.transform.translation.x;
//     float robot_y = tf.transform.translation.y;

//     auto raw_points = getFilteredLaserPoints();
//     // auto clustered_points = clusterPoints(raw_points, 0.3, 1);

//     std::vector<geometry_msgs::msg::Point> nearby_points;
//     // for (const auto &pt : clustered_points)
//     for (const auto &pt : raw_points)
//     {
//         float dx = pt.x - robot_x;
//         float dy = pt.y - robot_y;
//         if (std::hypot(dx, dy) <= 1.5)
//             nearby_points.push_back(pt);
//     }

//     if (nearby_points.empty()) {
//         RCLCPP_INFO(this->get_logger(), "No nearby unknown obstacles — skipping grid publish.");
//         return;
//     }

//     nav_msgs::msg::OccupancyGrid grid;
//     grid.header.stamp = this->now();
//     grid.header.frame_id = "map";
//     grid.info = map_info_;
//     grid.data.resize(grid.info.width * grid.info.height, -1);

//     for (const auto &pt : nearby_points)
//     {
//         int gx = static_cast<int>((pt.x - grid.info.origin.position.x) / grid.info.resolution);
//         int gy = static_cast<int>((pt.y - grid.info.origin.position.y) / grid.info.resolution);
//         int flipped_gy = grid.info.height - 1 - gy;

//         if (gx >= 0 && gx < static_cast<int>(grid.info.width) &&
//             flipped_gy >= 0 && flipped_gy < static_cast<int>(grid.info.height))
//         {
//             int index = flipped_gy * grid.info.width + gx;
//             grid.data[index] = 100;
//         }
//     }

//     unknown_grid_pub_->publish(grid);
//     RCLCPP_INFO(this->get_logger(), "✅ Published %zu clustered unknown obstacle(s) within 1m", nearby_points.size());
// }

// std::vector<geometry_msgs::msg::Point> ObstacleAvoidance::clusterPoints(
//     const std::vector<geometry_msgs::msg::Point> &points, double cluster_radius, int min_cluster_size)
// {
//     std::vector<geometry_msgs::msg::Point> clustered;
//     std::vector<bool> visited(points.size(), false);

//     for (size_t i = 0; i < points.size(); ++i)
//     {
//         if (visited[i])
//             continue;

//         std::vector<size_t> cluster_indices;
//         cluster_indices.push_back(i);
//         visited[i] = true;

//         geometry_msgs::msg::Point centroid = points[i];
//         int cluster_size = 1;

//         for (size_t j = i + 1; j < points.size(); ++j)
//         {
//             if (visited[j])
//                 continue;

//             double dx = points[i].x - points[j].x;
//             double dy = points[i].y - points[j].y;
//             if (std::hypot(dx, dy) < cluster_radius)
//             {
//                 cluster_indices.push_back(j);
//                 visited[j] = true;

//                 centroid.x += points[j].x;
//                 centroid.y += points[j].y;
//                 cluster_size++;
//             }
//         }

//         if (cluster_size >= min_cluster_size)
//         {
//             centroid.x /= cluster_size;
//             centroid.y /= cluster_size;
//             clustered.push_back(centroid);
//         }
//     }

//     return clustered;
// }
#include "obstacle_avoidance.hpp"
#include <cmath>
#include <opencv2/opencv.hpp>

ObstacleAvoidance::ObstacleAvoidance() : Node("obstacle_avoidance")
{
    static_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/static_occupancy_grid", 1, std::bind(&ObstacleAvoidance::staticMapCallback, this, std::placeholders::_1));

    local_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/local_costmap/costmap", 1, std::bind(&ObstacleAvoidance::localCostmapCallback, this, std::placeholders::_1));

    unknown_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/updated_occupancy_grid", 10);

    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ObstacleAvoidance::publishUnknownObstacles, this));

    visual_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&ObstacleAvoidance::displayUnknownMap, this));

    static_map_ready_ = false;
    local_costmap_ready_ = false;

    static_grid_client_ = this->create_client<std_srvs::srv::Trigger>("/request_static_grid");

    request_timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        [this]()
        {
            this->requestStaticGridOnce();
            this->request_timer_->cancel(); // one-shot
        });

    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&ObstacleAvoidance::laserCallback, this, std::placeholders::_1));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ObstacleAvoidance::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    latest_scan_ = *msg;
}

void ObstacleAvoidance::requestStaticGridOnce()
{
    if (!static_grid_client_->wait_for_service(std::chrono::seconds(2)))
    {
        RCLCPP_WARN(this->get_logger(), "⚠️ Service /request_static_grid not available.");
        return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

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

// void ObstacleAvoidance::staticMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
// {
//     static_map_ = *msg;
//     if (!static_map_ready_)
//         RCLCPP_INFO(this->get_logger(), "✅ Static map received (first time)");
//     static_map_ready_ = true;
// }

void ObstacleAvoidance::staticMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    static_map_ = *msg;

    // === One-time TTL buffer initialization ===
    const size_t grid_size = static_map_.info.width * static_map_.info.height;

    if (last_seen_.size() != grid_size)
    {
        last_seen_.clear();
        last_seen_.resize(grid_size, this->now());
        RCLCPP_INFO(this->get_logger(), "🕒 Initialized TTL timestamp buffer for %zu cells", grid_size);
    }

    if (!static_map_ready_)
        RCLCPP_INFO(this->get_logger(), "✅ Static map received (first time)");

    static_map_ready_ = true;
}

void ObstacleAvoidance::localCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    local_costmap_ = *msg;
    if (!local_costmap_ready_)
        RCLCPP_INFO(this->get_logger(), "📍 Local costmap received (first time)");
    local_costmap_ready_ = true;
}

void ObstacleAvoidance::displayUnknownMap()
{
    if (!static_map_ready_ || !local_costmap_ready_)
        return;

    int width = local_costmap_.info.width;
    int height = local_costmap_.info.height;
    cv::Mat unknown_img(height, width, CV_8UC1);

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            int flipped_y = height - 1 - y;
            int idx = flipped_y * width + x;

            int local_val = local_costmap_.data[idx];
            int static_val = static_map_.data[idx];

            if (local_val == 100 && static_val != 100)
                unknown_img.at<uchar>(y, x) = 0; // Black: new obstacle
            else
                unknown_img.at<uchar>(y, x) = 255; // White: ignore
        }
    }

    cv::resize(unknown_img, unknown_img, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST);
    cv::imshow("🔍 Unknown Obstacles View (before publish)", unknown_img);
    cv::waitKey(1);
}

// void ObstacleAvoidance::publishUnknownObstacles()
// {

//     const double TTL_SECONDS = 2.0;  // Obstacle cells expire after 2 seconds

//     if (!static_map_ready_ || !local_costmap_ready_) {
//         RCLCPP_WARN(this->get_logger(),
//             "❌ Skipping publish — maps not ready (static: %s, local: %s)",
//             static_map_ready_ ? "✅" : "❌",
//             local_costmap_ready_ ? "✅" : "❌");
//         return;
//     }

//     const auto &local_info = local_costmap_.info;
//     const auto &static_info = static_map_.info;

//     // Create a blank grid matching the static map
//     nav_msgs::msg::OccupancyGrid unknown_grid;
//     unknown_grid.header.stamp = this->now();
//     unknown_grid.header.frame_id = "map";
//     unknown_grid.info = static_info;
//     unknown_grid.data.resize(static_info.width * static_info.height, -1); // initialize all to unknown

//     for (int y = 0; y < local_info.height; ++y)
//     {
//         for (int x = 0; x < local_info.width; ++x)
//         {
//             int idx = y * local_info.width + x;
//             if (local_costmap_.data[idx] != 100)
//                 continue;

//             // Convert this local costmap cell to world coordinates
//             float wx = x * local_info.resolution + local_info.origin.position.x;
//             float wy = y * local_info.resolution + local_info.origin.position.y;

//             // Convert world coordinates to static map indices
//             int sx = static_cast<int>((wx - static_info.origin.position.x) / static_info.resolution);
//             int sy = static_cast<int>((wy - static_info.origin.position.y) / static_info.resolution);

//             if (sx < 0 || sx >= static_info.width || sy < 0 || sy >= static_info.height)
//                 continue;

//            int flipped_sy = static_info.height - 1 - sy;
//            int sidx = flipped_sy * static_info.width + sx;

//             // If not already occupied in the static map, mark it
//             if (static_map_.data[sidx] != 100)
//                 unknown_grid.data[sidx] = 100;
//         }
//     }

//     // unknown_grid_pub_->publish(unknown_grid);
//     RCLCPP_INFO(this->get_logger(), "✅ Published updated occupancy grid from local costmap comparison");
// }

// void ObstacleAvoidance::publishUnknownObstacles()
// {
//     const double TTL_SECONDS = 2.0;  // Obstacle cells expire after 2 seconds

//     if (!static_map_ready_ || !local_costmap_ready_) {
//         RCLCPP_WARN(this->get_logger(),
//             "❌ Skipping publish — maps not ready (static: %s, local: %s)",
//             static_map_ready_ ? "✅" : "❌",
//             local_costmap_ready_ ? "✅" : "❌");
//         return;
//     }

//     const auto &local_info = local_costmap_.info;
//     const auto &static_info = static_map_.info;

//     // Create a blank grid matching the static map
//     nav_msgs::msg::OccupancyGrid unknown_grid;
//     unknown_grid.header.stamp = this->now();
//     unknown_grid.header.frame_id = "map";
//     unknown_grid.info = static_info;
//     unknown_grid.data.resize(static_info.width * static_info.height, -1); // All unknown

//     // === Step 1: Mark new dynamic obstacle cells (cost == 100 only) ===
//     for (int y = 0; y < local_info.height; ++y)
//     {
//         for (int x = 0; x < local_info.width; ++x)
//         {
//             int local_idx = y * local_info.width + x;
//             if (local_costmap_.data[local_idx] != 100)
//                 continue;  // Only consider fully certain obstacle cells

//             // Convert local costmap cell to world coordinates
//             float wx = x * local_info.resolution + local_info.origin.position.x;
//             float wy = y * local_info.resolution + local_info.origin.position.y;

//             // Convert world coordinates to static map indices
//             int sx = static_cast<int>((wx - static_info.origin.position.x) / static_info.resolution);
//             int sy = static_cast<int>((wy - static_info.origin.position.y) / static_info.resolution);

//             if (sx < 0 || sx >= static_info.width || sy < 0 || sy >= static_info.height)
//                 continue;

//             int flipped_sy = static_info.height - 1 - sy;
//             int sidx = flipped_sy * static_info.width + sx;

//             // Only mark if this cell isn't part of the static map
//             if (static_map_.data[sidx] != 100)
//             {
//                 unknown_grid.data[sidx] = 100;
//                 last_seen_[sidx] = this->now();  // 🕒 Update last seen timestamp
//             }
//         }
//     }

//     // === Step 2: TTL decay — clear stale cells ===
//     int cleared = 0;
//     for (size_t i = 0; i < unknown_grid.data.size(); ++i)
//     {
//         rclcpp::Duration age = this->now() - last_seen_[i];

//         // If this cell hasn't been updated within TTL, clear it
//         if (age.seconds() > TTL_SECONDS)
//         {
//             unknown_grid.data[i] = -1;
//             cleared++;
//         }
//         // NOTE: We do NOT restore any obstacle cell unless it's freshly observed this cycle
//     }

//     unknown_grid_pub_->publish(unknown_grid);
//     RCLCPP_INFO(this->get_logger(), "✅ Published updated occupancy grid (TTL cleared %d cells)", cleared);
// }

/// ttl and laserscan together
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

void ObstacleAvoidance::publishUnknownObstacles()
{
    const double TTL_SECONDS = 2.0;
    const double LASER_CONFIRMATION_RADIUS = 0.1;

    if (!static_map_ready_ || !local_costmap_ready_ || latest_scan_.ranges.empty()) {
        RCLCPP_WARN(this->get_logger(),
            "❌ Skipping publish — maps or scan not ready (static: %s, local: %s, scan: %s)",
            static_map_ready_ ? "✅" : "❌",
            local_costmap_ready_ ? "✅" : "❌",
            latest_scan_.ranges.empty() ? "❌" : "✅");
        return;
    }

    const auto &local_info = local_costmap_.info;
    const auto &static_info = static_map_.info;
    const size_t grid_size = static_info.width * static_info.height;

    if (last_seen_.size() != grid_size) {
        RCLCPP_ERROR(this->get_logger(), "❌ last_seen_ size mismatch: expected %zu, got %zu", grid_size, last_seen_.size());
        return;
    }

    nav_msgs::msg::OccupancyGrid unknown_grid;
    unknown_grid.header.stamp = this->now();
    unknown_grid.header.frame_id = "map";
    unknown_grid.info = static_info;
    unknown_grid.data.assign(grid_size, -1);

    // === Transform LaserScan points into map frame ===
    std::vector<geometry_msgs::msg::Point> laser_points;
    try {
        geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform("map", latest_scan_.header.frame_id, tf2::TimePointZero);
        for (size_t i = 0; i < latest_scan_.ranges.size(); ++i) {
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
                laser_points.push_back(pt_global.point);
            } catch (const tf2::TransformException &ex) {
                continue;
            }
        }
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF2 Error: %s", ex.what());
    }

    // === Use laser scan points to validate costmap obstacles ===
    int marked = 0;
    for (int y = 0; y < local_info.height; ++y) {
        for (int x = 0; x < local_info.width; ++x) {
            int local_idx = y * local_info.width + x;

            float wx = x * local_info.resolution + local_info.origin.position.x;
            float wy = y * local_info.resolution + local_info.origin.position.y;

            int sx = static_cast<int>((wx - static_info.origin.position.x) / static_info.resolution);
            int sy = static_cast<int>((wy - static_info.origin.position.y) / static_info.resolution);

            if (sx < 0 || sx >= static_info.width || sy < 0 || sy >= static_info.height)
                continue;

            int flipped_sy = static_info.height - 1 - sy;
            int sidx = flipped_sy * static_info.width + sx;

            if (sidx < 0 || static_cast<size_t>(sidx) >= grid_size) {
                RCLCPP_ERROR(this->get_logger(), "❌ Skipped invalid sidx=%d (grid size=%zu)", sidx, grid_size);
                continue;
            }

            bool is_obstacle = local_costmap_.data[local_idx] == 100;

            if (is_obstacle && static_map_.data[sidx] != 100) {
                // Check if any laser point confirms this obstacle cell
                bool confirmed_by_laser = false;
                for (const auto &pt : laser_points) {
                    float dx = pt.x - wx;
                    float dy = pt.y - wy;
                    if (std::hypot(dx, dy) < LASER_CONFIRMATION_RADIUS) {
                        confirmed_by_laser = true;
                        break;
                    }
                }

                if (!confirmed_by_laser)
                    continue;

                unknown_grid.data[sidx] = 100;
                last_seen_[sidx] = this->now();
                marked++;
            }
        }
    }

    int cleared = 0;
    rclcpp::Time now = this->now();
    for (size_t i = 0; i < grid_size; ++i) {
        rclcpp::Duration age = now - last_seen_[i];
        if (age.seconds() > TTL_SECONDS) {
            unknown_grid.data[i] = -1;
            cleared++;
        }
    }

    unknown_grid_pub_->publish(unknown_grid);
    // RCLCPP_INFO(this->get_logger(), "✅ Published dynamic occupancy grid | Marked: %d | Cleared TTL: %d | Laser pts: %zu", marked, cleared, laser_points.size());
}
