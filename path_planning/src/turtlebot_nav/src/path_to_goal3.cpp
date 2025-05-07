#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>

using std::placeholders::_1;

struct MapMeta {
    std::string image_path;
    float resolution;
    float origin_x;
    float origin_y;
    float occupied_thresh;
    float free_thresh;
    bool negate;
};

struct Point {
    int x, y;
    bool operator==(const Point &other) const {
        return x == other.x && y == other.y;
    }
};

namespace std {
    template<>
    struct hash<Point> {
        std::size_t operator()(const Point& p) const {
            return std::hash<int>()(p.x) ^ std::hash<int>()(p.y);
        }
    };
}

struct CompareFScore {
    bool operator()(const std::tuple<float, Point>& a, const std::tuple<float, Point>& b) const {
        return std::get<0>(a) > std::get<0>(b);
    }
};

class PathToGoalClient : public rclcpp::Node {
public:
    PathToGoalClient() : Node("path_planner") {
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("computed_path", 10);
        goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pose_topic", 10, std::bind(&PathToGoalClient::pose_callback, this, _1));
        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&PathToGoalClient::odometry_callback, this, _1));

        map_meta_ = load_map_yaml("gallery_map.yaml");
        int inflation_radius = 3;  // Adjust this value to set the safety margin
        auto [inflated_grid, visualization_grid] = load_map_image(map_meta_.image_path, map_meta_, inflation_radius);

        occupancy_grid_ = inflated_grid;  // Use the inflated grid for pathfinding
        visualization_grid_ = visualization_grid;  // Use the visualization grid for marking

        cv::imwrite("debug_grid_with_inflation.png", visualization_grid_);
        RCLCPP_INFO(this->get_logger(), "Saved occupancy grid with inflation to debug_grid_with_inflation.png");
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    MapMeta map_meta_;
    cv::Mat occupancy_grid_;
    cv::Mat visualization_grid_;
    geometry_msgs::msg::Pose current_pose_;

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg->pose.pose;
    }

    MapMeta load_map_yaml(const std::string& yaml_path) {
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

    std::pair<cv::Mat, cv::Mat> load_map_image(const std::string& pgm_path, const MapMeta& meta, int inflation_radius) {
        cv::Mat image = cv::imread(pgm_path, cv::IMREAD_GRAYSCALE);
        image = 255 - image;  // Always invert the image
        cv::Mat grid(image.rows, image.cols, CV_8UC1);
        for (int y = 0; y < image.rows; ++y) {
            for (int x = 0; x < image.cols; ++x) {
                float val = image.at<uchar>(y, x) / 255.0f;  // Normalize to [0, 1]
                if (val > meta.occupied_thresh) grid.at<uchar>(y, x) = 255;  // White for obstacles
                else if (val < meta.free_thresh) grid.at<uchar>(y, x) = 0;   // Black for free space
                else grid.at<uchar>(y, x) = 127;  // Gray for unknown space
            }
        }

        // Inflate obstacles
        cv::Mat inflated_grid = grid.clone();
        if (inflation_radius > 0) {
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                       cv::Size(2 * inflation_radius + 1, 2 * inflation_radius + 1));
            cv::dilate(grid, inflated_grid, kernel);
        }

        // Create a visualization grid
        cv::Mat visualization_grid;
        cv::cvtColor(grid, visualization_grid, cv::COLOR_GRAY2BGR);

        // Mark inflated areas in yellow
        for (int y = 0; y < inflated_grid.rows; ++y) {
            for (int x = 0; x < inflated_grid.cols; ++x) {
                if (inflated_grid.at<uchar>(y, x) == 255 && grid.at<uchar>(y, x) != 255) {
                    visualization_grid.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 255);  // Yellow for inflated areas
                }
            }
        }

        return {inflated_grid, visualization_grid};  // Return both grids
    }

    Point world_to_grid(float wx, float wy) {
        int gx = static_cast<int>((wx - map_meta_.origin_x) / map_meta_.resolution);
        int gy = static_cast<int>((map_meta_.origin_y + map_meta_.resolution * occupancy_grid_.rows - wy) / map_meta_.resolution);
        return {gx, gy};
    }

    geometry_msgs::msg::PoseStamped grid_to_pose(int gx, int gy) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = gx * map_meta_.resolution + map_meta_.origin_x + map_meta_.resolution / 2.0f;
        pose.pose.position.y = map_meta_.origin_y + map_meta_.resolution * occupancy_grid_.rows - (gy * map_meta_.resolution + map_meta_.resolution / 2.0f);
        pose.pose.orientation.w = 1.0;
        return pose;
    }

    float heuristic(const Point& a, const Point& b) {
        RCLCPP_DEBUG(this->get_logger(), "Heuristic from (%d, %d) to (%d, %d): %.2f", a.x, a.y, b.x, b.y, heuristic(a, b));
        return std::hypot(a.x - b.x, a.y - b.y);
    }

    std::vector<Point> get_neighbors(const Point& p) {
        std::vector<Point> neighbors;
        std::vector<std::pair<int, int>> deltas = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
        for (const auto& d : deltas) {
            int nx = p.x + d.first;
            int ny = p.y + d.second;
            if (nx >= 0 && ny >= 0 && nx < occupancy_grid_.cols && ny < occupancy_grid_.rows) {
                if (occupancy_grid_.at<uchar>(ny, nx) < 50) {
                    neighbors.push_back({nx, ny});
                }
            }
        }
        return neighbors;
    }

    std::vector<Point> a_star(const Point& start, const Point& goal) {
        std::priority_queue<std::tuple<float, Point>, std::vector<std::tuple<float, Point>>, CompareFScore> open_list;
        open_list.push({0.0f, start});
        std::unordered_map<Point, Point> came_from;
        std::unordered_map<Point, float> g_score;
        g_score[start] = 0.0f;

        while (!open_list.empty()) {
            Point current = std::get<1>(open_list.top());
            open_list.pop();

            RCLCPP_DEBUG(this->get_logger(), "Exploring node: (%d, %d)", current.x, current.y);

            if (current == goal) {
                std::vector<Point> path;
                while (came_from.find(current) != came_from.end()) {
                    path.push_back(current);
                    current = came_from[current];
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            for (const auto& neighbor : get_neighbors(current)) {
                float tentative_g_score = g_score[current] + heuristic(current, neighbor);
                RCLCPP_DEBUG(this->get_logger(), "Neighbor: (%d, %d), Tentative g_score: %.2f", neighbor.x, neighbor.y, tentative_g_score);
                if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) {
                    came_from[neighbor] = current;
                    g_score[neighbor] = tentative_g_score;
                    float f_score = tentative_g_score + heuristic(neighbor, goal);
                    open_list.push({f_score, neighbor});
                }
            }
        }
        return {};  // No path found
    }

    void publish_path(const std::vector<Point>& path) {
        nav_msgs::msg::Path ros_path;
        ros_path.header.frame_id = "map";  // Make sure the frame_id matches your setup
        for (const auto& point : path) {
            ros_path.poses.push_back(grid_to_pose(point.x, point.y));
        }
        path_publisher_->publish(ros_path);
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received goal pose: (%.2f, %.2f)", msg->pose.position.x, msg->pose.position.y);
        RCLCPP_INFO(this->get_logger(), "Current pose: (%.2f, %.2f)", current_pose_.position.x, current_pose_.position.y);

        // Convert odometry and goal pose to grid coordinates
        Point start = world_to_grid(current_pose_.position.x, current_pose_.position.y);
        Point goal = world_to_grid(msg->pose.position.x, msg->pose.position.y);

        RCLCPP_INFO(this->get_logger(), "Start grid: (%d, %d), Goal grid: (%d, %d)", start.x, start.y, goal.x, goal.y);

        // Check if start or goal is in an occupied cell
        RCLCPP_INFO(this->get_logger(), "Start grid value: %d", occupancy_grid_.at<uchar>(start.y, start.x));
        // if (occupancy_grid_.at<uchar>(start.y, start.x) == 255) {
        //     RCLCPP_WARN(this->get_logger(), "Start position is in an occupied cell.");
        //     return;
        // }
        // if (occupancy_grid_.at<uchar>(goal.y, goal.x) >= 50) {
        //     RCLCPP_WARN(this->get_logger(), "Goal position is in an occupied cell.");
        //     return;
        // }

        // Call A* to find path
        std::vector<Point> path = a_star(start, goal);

        if (path.empty()) {
            RCLCPP_WARN(this->get_logger(), "No valid path found.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Path found with %zu points.", path.size());
        }

        // Clone the visualization grid for marking
        cv::Mat marked_grid = visualization_grid_.clone();

        // Mark the start and goal positions on the visualization grid
        mark_position(marked_grid, start, cv::Vec3b(255, 0, 0));  // Red for start
        mark_position(marked_grid, goal, cv::Vec3b(0, 0, 255));  // Blue for goal

        // Mark the path on the visualization grid
        if (!path.empty()) {
            mark_path(marked_grid, path, cv::Vec3b(0, 255, 0));  // Green for path
        }

        // Save the updated visualization grid
        cv::imwrite("debug_grid_with_positions.png", marked_grid);
        RCLCPP_INFO(this->get_logger(), "Saved updated occupancy grid with start, goal, path, and inflation to debug_grid_with_positions.png");

        // Publish the path
        if (!path.empty()) {
            publish_path(path);
        }
    }

    void mark_position(cv::Mat& grid, const Point& position, const cv::Vec3b& color) {
        // Convert the grid to a 3-channel color image if not already
        if (grid.channels() == 1) {
            cv::cvtColor(grid, grid, cv::COLOR_GRAY2BGR);
        }

        // Mark the position with the specified color
        if (position.x >= 0 && position.x < grid.cols &&
            position.y >= 0 && position.y < grid.rows) {
            grid.at<cv::Vec3b>(position.y, position.x) = color;
        }
    }

    void mark_path(cv::Mat& grid, const std::vector<Point>& path, const cv::Vec3b& color) {
        // Ensure the grid is a 3-channel color image
        if (grid.channels() == 1) {
            cv::cvtColor(grid, grid, cv::COLOR_GRAY2BGR);
        }

        // Mark each point in the path with the specified color
        for (const auto& point : path) {
            if (grid.at<cv::Vec3b>(point.y, point.x) != cv::Vec3b(255, 0, 0) &&  // Avoid overriding start (red)
                grid.at<cv::Vec3b>(point.y, point.x) != cv::Vec3b(0, 0, 255)) {  // Avoid overriding goal (blue)
                grid.at<cv::Vec3b>(point.y, point.x) = color;  // Green for path
            }
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathToGoalClient>());
    rclcpp::shutdown();
    return 0;
}
