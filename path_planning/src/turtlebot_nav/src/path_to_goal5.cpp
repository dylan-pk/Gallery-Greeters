#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>

#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"

#include <chrono>
#include <thread>

// swithching to service as static grid isnt always sending
#include "std_srvs/srv/trigger.hpp"

using std::placeholders::_1;

struct MapMeta
{
    std::string image_path;
    float resolution;
    float origin_x;
    float origin_y;
    float occupied_thresh;
    float free_thresh;
    bool negate;
};

struct Point
{
    int x, y;
    bool operator==(const Point &other) const
    {
        return x == other.x && y == other.y;
    }
};

namespace std
{
    template <>
    struct hash<Point>
    {
        std::size_t operator()(const Point &p) const
        {
            return std::hash<int>()(p.x) ^ std::hash<int>()(p.y);
        }
    };
}

struct CompareFScore
{
    bool operator()(const std::tuple<float, Point> &a, const std::tuple<float, Point> &b) const
    {
        return std::get<0>(a) > std::get<0>(b);
    }
};

class PathToGoalClient : public rclcpp::Node
{
public:
    PathToGoalClient() : Node("path_planner")
    {
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("computed_path", 10);
        goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pose_topic", 10, std::bind(&PathToGoalClient::pose_callback, this, _1));
        // odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        //     "odom", 10, std::bind(&PathToGoalClient::odometry_callback, this, _1));
        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&PathToGoalClient::odometry_callback, this, _1));
        map_meta_ = load_map_yaml("real_map.yaml");
        // map_meta_ = load_map_yaml("gallery_map.yaml");
        // int inflation_radius = 4;//2.75;

        float inflation_radius_m = 0.15; // meters
        int inflation_radius = static_cast<int>(std::ceil(inflation_radius_m / map_meta_.resolution));

        // auto [inflated_grid, visualization_grid] = load_map_image(map_meta_.image_path, map_meta_, inflation_radius);

        auto [raw_grid, inflated_grid, visualization] = load_map_image(map_meta_.image_path, map_meta_, inflation_radius);
        raw_static_grid_ = raw_grid;
        inflated_static_grid_ = inflated_grid;
        occupancy_grid_ = inflated_grid.clone(); // this is the grid A* uses and gets updated
        visualization_grid_ = visualization;

        // occupancy_grid_ = inflated_grid;
        // visualization_grid_ = visualization_grid;

        cv::imwrite("debug_grid_with_inflation.png", visualization_grid_);
        RCLCPP_INFO(this->get_logger(), "Saved occupancy grid with inflation to debug_grid_with_inflation.png");

        obstacle_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/known_static_obstacles", 10);
        extract_static_obstacles();

        dynamic_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/updated_occupancy_grid", 10,
            std::bind(&PathToGoalClient::dynamic_grid_callback, this, _1));

        static_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/static_occupancy_grid", 1);
        // occupancy_timer_ = this->create_wall_timer(
        //     std::chrono::seconds(3),
        //     std::bind(&PathToGoalClient::publish_static_grid, this));

        static_grid_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/request_static_grid",
            std::bind(&PathToGoalClient::handle_static_grid_request, this, std::placeholders::_1, std::placeholders::_2));

        initialpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        initialpose_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&PathToGoalClient::publish_initial_pose, this));

        // === Create a window to show the grid live ===
        cv::namedWindow("Live Occupancy Grid", cv::WINDOW_NORMAL);
        live_display_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&PathToGoalClient::update_live_display, this));

        base_grid_ = raw_static_grid_.clone(); // base for applying dynamic obstacles
                                               // keep the clean base map

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&PathToGoalClient::scan_callback, this, _1));

        path_vector_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/sentry_path", 10, std::bind(&PathToGoalClient::sentry_path_callback, this, _1));

        mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/robot_mode", 10, std::bind(&PathToGoalClient::mode_callback, this, std::placeholders::_1));

        interrupt_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/interrupt_signal", 10, std::bind(&PathToGoalClient::interrupt_callback, this, std::placeholders::_1));

        live_path_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&PathToGoalClient::update_live_path_display, this));
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    MapMeta map_meta_;
    cv::Mat occupancy_grid_;
    cv::Mat visualization_grid_;
    geometry_msgs::msg::Pose current_pose_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr dynamic_grid_sub_;

    geometry_msgs::msg::PoseStamped last_goal_;
    bool has_last_goal_ = false;

    rclcpp::TimerBase::SharedPtr occupancy_timer_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr static_grid_pub_;

    // added for localisation
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
    rclcpp::TimerBase::SharedPtr initialpose_timer_;
    bool initialpose_sent_ = false;

    // cv::Mat raw_static_grid_;     // original static map before inflation
    cv::Mat base_grid_;             // static + dynamic obstacles before inflation
                                    // keeps raw uninflated static + dynamic obstacles
    cv::Mat raw_static_grid_;       // from PGMap
    cv::Mat inflated_static_grid_;  // static + static inflation only
    cv::Mat dynamic_overlay_grid_;  // raw dynamic obstacles
    cv::Mat inflated_dynamic_grid_; // dynamic + inflated
    // cv::Mat occupancy_grid_;            // final grid used for A*

    std::vector<Point> last_computed_path_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_vector_sub_;
    bool sentry_mode_ = false;
    bool table_mode_ = false;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
    int received_int_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr interrupt_sub_;
    bool interrupted_ = false;

    int mismatch_count_ = 0;
    const int mismatch_threshold_ = 2;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr static_grid_service_;

    void handle_static_grid_request(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request; // avoid unused variable warning

        publish_static_grid();

        response->success = true;
        response->message = "Static occupancy grid published.";
        RCLCPP_INFO(this->get_logger(), "✅ Static grid published via service call.");
    }

    void mode_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        received_int_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received control mode: %d", received_int_);

        if (received_int_ == 1)
        {
            sentry_mode_ = true;
            RCLCPP_INFO(this->get_logger(), "Sentry Mode Activated");
        }
        else if (received_int_ == 4)
        {
            table_mode_ = true;
            RCLCPP_INFO(this->get_logger(), "Table Mode Activated");
        }
        else
        {
            sentry_mode_ = false;
            table_mode_ = false;
            RCLCPP_INFO(this->get_logger(), "Sentry Mode Deactivated");
        }
    }


    void publish_initial_pose()
    {
        if (initialpose_sent_)
            return;

        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map"; // still map, as AMCL needs it in map frame

        // Use the latest odometry pose as the initial estimate
        msg.pose.pose = current_pose_;
        msg.pose.pose.position.x = msg.pose.pose.position.x + 0.2;
        msg.pose.pose.position.y = msg.pose.pose.position.y + 0.2;

        // Optional: Adjust covariance (still needed)
        msg.pose.covariance[0] = 0.25;    // x variance
        msg.pose.covariance[7] = 0.25;    // y variance
        msg.pose.covariance[35] = 0.0685; // yaw variance

        initialpose_pub_->publish(msg);
        initialpose_sent_ = true;

        RCLCPP_INFO(this->get_logger(), "📍 Published initial pose from odometry.");
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        if (!isScanMatchingMap(*scan_msg, current_pose_, occupancy_grid_, map_meta_))
        {
            mismatch_count_++;
            RCLCPP_WARN(this->get_logger(), "Laser scan mismatch detected (%d/5)", mismatch_count_);
        }
        else
        {
            mismatch_count_ = 0; // reset if alignment is good
        }

        if (mismatch_count_ >= mismatch_threshold_)
        {
            RCLCPP_ERROR(this->get_logger(), "5 consecutive mismatches — correcting pose estimation.");

            geometry_msgs::msg::PoseWithCovarianceStamped corrected_pose;
            corrected_pose.header.stamp = this->get_clock()->now();
            corrected_pose.header.frame_id = "map";

            corrected_pose.pose.pose.position = current_pose_.position;
            corrected_pose.pose.pose.orientation = current_pose_.orientation;

            corrected_pose.pose.covariance[0] = 0.5;  // 0.05;
            corrected_pose.pose.covariance[7] = 0.5;  // 0.05;
            corrected_pose.pose.covariance[35] = 0.1; // 0.02;

            initialpose_pub_->publish(corrected_pose);
            mismatch_count_ = 0;
        }
    }

    bool isScanMatchingMap(const sensor_msgs::msg::LaserScan &scan,
                           const geometry_msgs::msg::Pose &pose,
                           const cv::Mat &occupancy_map,
                           const MapMeta &map_meta)
    {
        int hit_count = 0;
        int total = 0;

        double angle = scan.angle_min;
        for (const auto &range : scan.ranges)
        {
            if (range < scan.range_min || range > scan.range_max)
            {
                angle += scan.angle_increment;
                continue;
            }

            // Transform scan point to world
            double lx = pose.position.x + range * std::cos(angle + getYaw(pose.orientation));
            double ly = pose.position.y + range * std::sin(angle + getYaw(pose.orientation));

            // Convert to grid
            int gx = static_cast<int>((lx - map_meta.origin_x) / map_meta.resolution);
            int gy = static_cast<int>((map_meta.origin_y + map_meta.resolution * occupancy_map.rows - ly) / map_meta.resolution);

            if (gx >= 0 && gx < occupancy_map.cols && gy >= 0 && gy < occupancy_map.rows)
            {
                if (occupancy_map.at<uchar>(gy, gx) >= 100)
                    hit_count++;
            }

            total++;
            angle += scan.angle_increment;
        }

        double hit_ratio = (total > 0) ? (double)hit_count / total : 0.0;
        return hit_ratio > 0.4; // threshold: at least 30% of points should hit known obstacles
    }

    double getYaw(const geometry_msgs::msg::Quaternion &q)
    {
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        return yaw;
    }

    ////

    void publish_static_grid()
    {
        nav_msgs::msg::OccupancyGrid grid_msg;
        grid_msg.header.stamp = this->now();
        grid_msg.header.frame_id = "map";

        grid_msg.info.resolution = map_meta_.resolution;
        grid_msg.info.width = occupancy_grid_.cols;
        grid_msg.info.height = occupancy_grid_.rows;
        grid_msg.info.origin.position.x = map_meta_.origin_x;
        grid_msg.info.origin.position.y = map_meta_.origin_y;
        grid_msg.info.origin.orientation.w = 1.0;

        grid_msg.data.resize(grid_msg.info.width * grid_msg.info.height);

        for (int y = 0; y < occupancy_grid_.rows; ++y)
        {
            for (int x = 0; x < occupancy_grid_.cols; ++x)
            {
                int idx = y * occupancy_grid_.cols + x;
                uchar val = occupancy_grid_.at<uchar>(y, x);
                if (val == 255)
                    grid_msg.data[idx] = 100;
                else if (val == 0)
                    grid_msg.data[idx] = 0;
                else
                    grid_msg.data[idx] = -1;
            }
        }

        static_grid_pub_->publish(grid_msg);
    }

    // ... (Existing member variables remain unchanged)
    rclcpp::TimerBase::SharedPtr live_display_timer_;

    void update_live_display()
    {
        cv::Mat debug_view;
        cv::cvtColor(occupancy_grid_, debug_view, cv::COLOR_GRAY2BGR);
        cv::imshow("Live Occupancy Grid", debug_view);
        cv::waitKey(1);
    }

    rclcpp::TimerBase::SharedPtr live_path_timer_;

    void update_live_path_display()
    {
        if (visualization_grid_.empty() || occupancy_grid_.empty() || inflated_dynamic_grid_.empty())
        {
            // RCLCPP_WARN(this->get_logger(), "❌ Skipping path display update: Grid not initialized.");
            return;
        }

        cv::Mat path_view = visualization_grid_.clone();

        if (path_view.channels() == 1)
            cv::cvtColor(path_view, path_view, cv::COLOR_GRAY2BGR);

        for (int y = 0; y < occupancy_grid_.rows; ++y)
        {
            for (int x = 0; x < occupancy_grid_.cols; ++x)
            {
                if (inflated_dynamic_grid_.at<uchar>(y, x) == 255)
                    path_view.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255); // Red
            }
        }

        for (const auto &pt : last_computed_path_)
        {
            if (pt.x >= 0 && pt.x < path_view.cols && pt.y >= 0 && pt.y < path_view.rows)
                path_view.at<cv::Vec3b>(pt.y, pt.x) = cv::Vec3b(0, 255, 0); // Green
        }

        Point robot_pt = world_to_grid(current_pose_.position.x, current_pose_.position.y);
        if (robot_pt.x >= 0 && robot_pt.x < path_view.cols && robot_pt.y >= 0 && robot_pt.y < path_view.rows)
            path_view.at<cv::Vec3b>(robot_pt.y, robot_pt.x) = cv::Vec3b(255, 255, 255); // White

        cv::imshow("Live Path View", path_view);
        cv::waitKey(1);
    }

    void dynamic_grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        if (msg->info.width != raw_static_grid_.cols || msg->info.height != raw_static_grid_.rows)
        {
            // RCLCPP_WARN(this->get_logger(), "Mismatch in grid dimensions. Skipping dynamic update.");
            return;
        }

        // Step 1: Convert to binary unknown obstacle mask
        dynamic_overlay_grid_ = cv::Mat::zeros(raw_static_grid_.size(), CV_8UC1);
        for (int y = 0; y < msg->info.height; ++y)
        {
            for (int x = 0; x < msg->info.width; ++x)
            {
                int idx = y * msg->info.width + x;
                if (msg->data[idx] == 100)
                    dynamic_overlay_grid_.at<uchar>(y, x) = 255;
            }
        }

        // Step 2: Inflate dynamic obstacles
        // int dynamic_inflation_radius = 3.5;  // adjustable

        float dynamic_inflation_radius_m = 0.35; // meters
        int dynamic_inflation_radius = static_cast<int>(std::ceil(dynamic_inflation_radius_m / map_meta_.resolution));
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                   cv::Size(2 * dynamic_inflation_radius + 1, 2 * dynamic_inflation_radius + 1));
        cv::dilate(dynamic_overlay_grid_, inflated_dynamic_grid_, kernel);

        // Step 3: Merge with static inflated map
        occupancy_grid_ = inflated_static_grid_.clone(); // start from static inflated
        occupancy_grid_ |= inflated_dynamic_grid_;       // merge dynamic

        // Optional: visual debug overlay
        for (int y = 0; y < occupancy_grid_.rows; ++y)
        {
            for (int x = 0; x < occupancy_grid_.cols; ++x)
            {
                if (inflated_dynamic_grid_.at<uchar>(y, x) == 255)
                    visualization_grid_.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255); // red for dynamic
            }
        }

        // RCLCPP_INFO(this->get_logger(), "✅ Dynamic grid merged and inflated with radius %.2f", dynamic_inflation_radius);

        if (!has_last_goal_)
            return;

        float dx = current_pose_.position.x - last_goal_.pose.position.x;
        float dy = current_pose_.position.y - last_goal_.pose.position.y;
        if (std::hypot(dx, dy) < 0.2)
        {
            RCLCPP_INFO(this->get_logger(), "Robot at goal, no replanning.");
            table_mode_ = false;
            return;
        }

        bool path_obstructed = false;
        for (const auto &pt : last_computed_path_)
        {
            if (pt.x >= 0 && pt.x < occupancy_grid_.cols &&
                pt.y >= 0 && pt.y < occupancy_grid_.rows &&
                inflated_dynamic_grid_.at<uchar>(pt.y, pt.x) == 255)
            {
                path_obstructed = true;
                break;
            }
        }

        if (path_obstructed)
        {
            RCLCPP_WARN(this->get_logger(), "⚠️ Path obstructed by new dynamic obstacle. Replanning...");
            std::this_thread::sleep_for(std::chrono::seconds(2));
            pose_callback(std::make_shared<geometry_msgs::msg::PoseStamped>(last_goal_));
        }
    }

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
    }

    MapMeta load_map_yaml(const std::string &yaml_path)
    {
        YAML::Node config = YAML::LoadFile(yaml_path);
        MapMeta meta;
        meta.image_path = config["image"].as<std::string>();
        meta.resolution = config["resolution"].as<float>();
        meta.origin_x = config["origin"][0].as<float>();
        meta.origin_y = config["origin"][1].as<float>();
        meta.occupied_thresh = config["occupied_thresh"].as<float>();
        meta.free_thresh = config["free_thresh"].as<float>();
        meta.negate = config["negate"].as<int>() != 0;
        RCLCPP_INFO(this->get_logger(), "Loaded map: %s", yaml_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Map resolution: %.2f, Origin: (%.2f, %.2f)", meta.resolution, meta.origin_x, meta.origin_y);
        return meta;
    }

    std::tuple<cv::Mat, cv::Mat, cv::Mat> load_map_image(const std::string &pgm_path, const MapMeta &meta, int static_inflation_radius)
    {
        // === Step 1: Load and invert the PGM image ===
        cv::Mat image = cv::imread(pgm_path, cv::IMREAD_GRAYSCALE);
        if (image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to load map image: %s", pgm_path.c_str());
            throw std::runtime_error("Map image not found");
        }
        image = 255 - image;

        // === Step 2: Create the raw occupancy grid ===
        cv::Mat raw_grid(image.rows, image.cols, CV_8UC1);
        for (int y = 0; y < image.rows; ++y)
        {
            for (int x = 0; x < image.cols; ++x)
            {
                float val = image.at<uchar>(y, x) / 255.0f;
                if (val > meta.occupied_thresh)
                    raw_grid.at<uchar>(y, x) = 255;
                else if (val < meta.free_thresh)
                    raw_grid.at<uchar>(y, x) = 0;
                else
                    raw_grid.at<uchar>(y, x) = 127;
            }
        }

        // === Step 3: Inflate static obstacles ===
        cv::Mat inflated_grid = raw_grid.clone();
        if (static_inflation_radius > 0)
        {
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                       cv::Size(2 * static_inflation_radius + 1, 2 * static_inflation_radius + 1));
            cv::dilate(raw_grid, inflated_grid, kernel);
        }

        // === Step 4: Visualisation
        cv::Mat visualization_grid;
        cv::cvtColor(raw_grid, visualization_grid, cv::COLOR_GRAY2BGR);
        for (int y = 0; y < inflated_grid.rows; ++y)
        {
            for (int x = 0; x < inflated_grid.cols; ++x)
            {
                if (inflated_grid.at<uchar>(y, x) == 255 && raw_grid.at<uchar>(y, x) != 255)
                    visualization_grid.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 255); // Yellow = inflated
            }
        }

        return {raw_grid, inflated_grid, visualization_grid};
    }

    Point world_to_grid(float wx, float wy)
    {
        int gx = static_cast<int>((wx - map_meta_.origin_x) / map_meta_.resolution);
        int gy = static_cast<int>((map_meta_.origin_y + map_meta_.resolution * occupancy_grid_.rows - wy) / map_meta_.resolution);
        return {gx, gy};
    }

    geometry_msgs::msg::PoseStamped grid_to_pose(int gx, int gy)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = gx * map_meta_.resolution + map_meta_.origin_x + map_meta_.resolution / 2.0f;
        pose.pose.position.y = map_meta_.origin_y + map_meta_.resolution * occupancy_grid_.rows - (gy * map_meta_.resolution + map_meta_.resolution / 2.0f);
        pose.pose.orientation.w = 1.0;
        return pose;
    }

    float heuristic(const Point &a, const Point &b)
    {
        RCLCPP_DEBUG(this->get_logger(), "Heuristic from (%d, %d) to (%d, %d): %.2f", a.x, a.y, b.x, b.y, heuristic(a, b));
        return std::hypot(a.x - b.x, a.y - b.y);
    }

    std::vector<Point> get_neighbors(const Point &p)
    {
        std::vector<Point> neighbors;
        std::vector<std::pair<int, int>> deltas = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
        for (const auto &d : deltas)
        {
            int nx = p.x + d.first;
            int ny = p.y + d.second;
            if (nx >= 0 && ny >= 0 && nx < occupancy_grid_.cols && ny < occupancy_grid_.rows)
            {
                if (occupancy_grid_.at<uchar>(ny, nx) < 50)
                {
                    neighbors.push_back({nx, ny});
                }
            }
        }
        return neighbors;
    }

    std::vector<Point> a_star(const Point &start, const Point &goal)
    {
        std::priority_queue<std::tuple<float, Point>, std::vector<std::tuple<float, Point>>, CompareFScore> open_list;
        open_list.push({0.0f, start});
        std::unordered_map<Point, Point> came_from;
        std::unordered_map<Point, float> g_score;
        g_score[start] = 0.0f;

        while (!open_list.empty())
        {
            Point current = std::get<1>(open_list.top());
            open_list.pop();

            RCLCPP_DEBUG(this->get_logger(), "Exploring node: (%d, %d)", current.x, current.y);

            if (current == goal)
            {
                std::vector<Point> path;
                while (came_from.find(current) != came_from.end())
                {
                    path.push_back(current);
                    current = came_from[current];
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            for (const auto &neighbor : get_neighbors(current))
            {
                float tentative_g_score = g_score[current] + heuristic(current, neighbor);
                RCLCPP_DEBUG(this->get_logger(), "Neighbor: (%d, %d), Tentative g_score: %.2f", neighbor.x, neighbor.y, tentative_g_score);
                if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor])
                {
                    came_from[neighbor] = current;
                    g_score[neighbor] = tentative_g_score;
                    float f_score = tentative_g_score + heuristic(neighbor, goal);
                    open_list.push({f_score, neighbor});
                }
            }
        }
        return {}; // No path found
    }


    void publish_path(const std::vector<Point> &path)
    {
        nav_msgs::msg::Path ros_path;
        ros_path.header.frame_id = "map"; // Make sure the frame_id matches your setup
        for (const auto &point : path)
        {
            ros_path.poses.push_back(grid_to_pose(point.x, point.y));
        }
        path_publisher_->publish(ros_path);
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        last_goal_ = *msg;
        has_last_goal_ = true;

        RCLCPP_INFO(this->get_logger(), "Received goal pose: (%.2f, %.2f)", msg->pose.position.x, msg->pose.position.y);
        RCLCPP_INFO(this->get_logger(), "Current pose: (%.2f, %.2f)", current_pose_.position.x, current_pose_.position.y);

        Point start = world_to_grid(current_pose_.position.x, current_pose_.position.y);
        Point goal = world_to_grid(msg->pose.position.x, msg->pose.position.y);

        RCLCPP_INFO(this->get_logger(), "Start grid: (%d, %d), Goal grid: (%d, %d)", start.x, start.y, goal.x, goal.y);

        std::vector<Point> path = a_star(start, goal);
        last_computed_path_ = path;

        if (path.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No valid path found.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Path found with %zu points.", path.size());
        }

        cv::Mat marked_grid = visualization_grid_.clone();
        mark_position(marked_grid, start, cv::Vec3b(255, 0, 0)); // Red
        mark_position(marked_grid, goal, cv::Vec3b(0, 0, 255));  // Blue
        if (!path.empty())
        {
            mark_path(marked_grid, path, cv::Vec3b(0, 255, 0)); // Green
        }
        
        // Display the image in a window 
        cv::namedWindow("Path Visualization", cv::WINDOW_NORMAL); // Create a resizable window 
        // cv::resizeWindow("Path Visualization", 800, 600); // Set the window size to 800x600 pixels 
        int height = static_cast<int>(800 * (6.0 / 8));
        int width = static_cast<int>(600 * (6.0 / 8));
        cv::resizeWindow("Path Visualization", height, width);  
        cv::imshow("Path Visualization", marked_grid); 
        cv::waitKey(1); // Non-blocking, allows the program to continue 

        cv::imwrite("debug_grid_with_positions.png", marked_grid);

        if (!path.empty())
        {
            publish_path(path);
        }
    }

    void mark_position(cv::Mat &grid, const Point &position, const cv::Vec3b &color)
    {
        // Convert the grid to a 3-channel color image if not already
        if (grid.channels() == 1)
        {
            cv::cvtColor(grid, grid, cv::COLOR_GRAY2BGR);
        }

        // Mark the position with the specified color
        if (position.x >= 0 && position.x < grid.cols &&
            position.y >= 0 && position.y < grid.rows)
        {
            grid.at<cv::Vec3b>(position.y, position.x) = color;
        }
    }

    void mark_path(cv::Mat &grid, const std::vector<Point> &path, const cv::Vec3b &color)
    {
        // Ensure the grid is a 3-channel color image
        if (grid.channels() == 1)
        {
            cv::cvtColor(grid, grid, cv::COLOR_GRAY2BGR);
        }

        // Mark each point in the path with the specified color
        for (const auto &point : path)
        {
            if (grid.at<cv::Vec3b>(point.y, point.x) != cv::Vec3b(255, 0, 0) && // Avoid overriding start (red)
                grid.at<cv::Vec3b>(point.y, point.x) != cv::Vec3b(0, 0, 255))
            {                                                 // Avoid overriding goal (blue)
                grid.at<cv::Vec3b>(point.y, point.x) = color; // Green for path
            }
        }
    }

    std::vector<geometry_msgs::msg::Point> known_obstacles_world_;

    void extract_static_obstacles()
    {
        // Clear any previously stored obstacles
        known_obstacles_world_.clear();

        // Step 1: Extract obstacle points from the occupancy grid
        for (int y = 0; y < occupancy_grid_.rows; ++y)
        {
            for (int x = 0; x < occupancy_grid_.cols; ++x)
            {
                if (occupancy_grid_.at<uchar>(y, x) == 255)
                { // Obstacle cell
                    geometry_msgs::msg::Point p;
                    p.x = x * map_meta_.resolution + map_meta_.origin_x;
                    p.y = y * map_meta_.resolution + map_meta_.origin_y;
                    p.z = 0.0; // 2D plane
                    known_obstacles_world_.push_back(p);
                }
            }
        }

        // Step 2: Convert to PoseArray
        geometry_msgs::msg::PoseArray pose_array_msg;
        pose_array_msg.header.frame_id = "map";
        pose_array_msg.header.stamp = this->get_clock()->now();

        for (const auto &pt : known_obstacles_world_)
        {
            geometry_msgs::msg::Pose pose;
            pose.position = pt;
            pose.orientation.w = 1.0; // Neutral orientation (identity quaternion)
            pose_array_msg.poses.push_back(pose);
        }

        // Step 3: Publish
        obstacle_pub_->publish(pose_array_msg);

        RCLCPP_INFO(this->get_logger(), "Published %zu known static obstacles to /known_static_obstacles", pose_array_msg.poses.size());
    }

    int findClosestGoalIndex(const std::vector<geometry_msgs::msg::PoseStamped> &goals,
                             const geometry_msgs::msg::Pose &current_pose)
    {
        double min_dist = std::numeric_limits<double>::max();
        int closest_idx = 0;

        for (size_t i = 0; i < goals.size(); ++i)
        {
            double dx = goals[i].pose.position.x - current_pose.position.x;
            double dy = goals[i].pose.position.y - current_pose.position.y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_idx = static_cast<int>(i);
            }
        }
        return closest_idx;
    }

    void sentry_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty path.");
            return;
        }

        std::vector<geometry_msgs::msg::PoseStamped> sorted_goals;

        if (sentry_mode_)
        {
            // Reorder based on closest goal
            Point robot_pos = world_to_grid(current_pose_.position.x, current_pose_.position.y);
            const auto &poses = msg->poses;

            int closest_idx = 0;
            double min_dist = std::numeric_limits<double>::max();

            for (size_t i = 0; i < poses.size(); ++i)
            {
                Point goal_pt = world_to_grid(poses[i].pose.position.x, poses[i].pose.position.y);
                double dx = goal_pt.x - robot_pos.x;
                double dy = goal_pt.y - robot_pos.y;
                double dist = std::hypot(dx, dy);

                if (dist < min_dist)
                {
                    min_dist = dist;
                    closest_idx = i;
                }
            }

            for (size_t i = 0; i < poses.size(); ++i)
            {
                sorted_goals.push_back(poses[(closest_idx + i) % poses.size()]);
            }

            RCLCPP_INFO(this->get_logger(), "Reordered sentry goals from closest index %d", closest_idx);
        }
        else
        {
            sorted_goals = msg->poses; // use as-is
        }

        // Now convert goals into a full path
        std::vector<Point> full_path;
        for (size_t i = 0; i < sorted_goals.size() - 1; ++i)
        {
            Point start = world_to_grid(sorted_goals[i].pose.position.x, sorted_goals[i].pose.position.y);
            Point goal = world_to_grid(sorted_goals[i + 1].pose.position.x, sorted_goals[i + 1].pose.position.y);

            std::vector<Point> segment = a_star(start, goal);
            full_path.insert(full_path.end(), segment.begin(), segment.end());
        }

        if (full_path.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No valid path found.");
            return;
        }

        last_computed_path_ = full_path;
        RCLCPP_INFO(this->get_logger(), "Published path with %zu points", full_path.size());
        publish_path(full_path);
    }

    void interrupt_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        interrupted_ = msg->data;

        if (interrupted_ && sentry_mode_)
        {
            last_computed_path_.clear();
            has_last_goal_ = false;

            nav_msgs::msg::Path empty_path;
            empty_path.header.frame_id = "map";
            empty_path.header.stamp = this->get_clock()->now();
            path_publisher_->publish(empty_path);

            RCLCPP_WARN(this->get_logger(), "🚨 Interrupt received! Cleared last goal and path.");
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathToGoalClient>());
    rclcpp::shutdown();
    return 0;
}