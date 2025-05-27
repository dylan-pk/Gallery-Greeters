#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include <opencv2/opencv.hpp>
#include <vector>

class PathSubscriber : public rclcpp::Node {
public:
    PathSubscriber() : Node("path_subscriber") {
        // Create a subscription to the "computed_path" topic
        path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
            "computed_path", 10,
            std::bind(&PathSubscriber::path_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "PathSubscriber node has been started.");
    }

private:
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received path with %zu poses", msg->poses.size());
        if (msg->poses.empty()) return;

        // Create a blank image
        int img_size = 500;
        cv::Mat img(img_size, img_size, CV_8UC3, cv::Scalar(255,255,255));

        // Find min/max for scaling
        double min_x = msg->poses[0].pose.position.x, max_x = min_x;
        double min_y = msg->poses[0].pose.position.y, max_y = min_y;
        for (const auto& p : msg->poses) {
            min_x = std::min(min_x, p.pose.position.x);
            max_x = std::max(max_x, p.pose.position.x);
            min_y = std::min(min_y, p.pose.position.y);
            max_y = std::max(max_y, p.pose.position.y);
        }

        // Draw the path
        for (size_t i = 1; i < msg->poses.size(); ++i) {
            auto scale = [&](double v, double minv, double maxv) {
                return int((v - minv) / (maxv - minv + 1e-6) * (img_size-40) + 20);
            };
            int x1 = scale(msg->poses[i-1].pose.position.x, min_x, max_x);
            int y1 = img_size - scale(msg->poses[i-1].pose.position.y, min_y, max_y);
            int x2 = scale(msg->poses[i].pose.position.x, min_x, max_x);
            int y2 = img_size - scale(msg->poses[i].pose.position.y, min_y, max_y);
            cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0,0,255), 2);
        }

        cv::imshow("Path", img);
        cv::waitKey(1);
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathSubscriber>();
    rclcpp::spin(node);
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}