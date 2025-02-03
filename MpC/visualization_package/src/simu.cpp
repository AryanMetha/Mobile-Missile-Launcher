#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <vector>
#include <cmath>

using namespace std;

// ROS Node Class
class PathVisualizer : public rclcpp::Node {
public:
    PathVisualizer() : Node("path_visualizer") {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("robot_pose", 10);

        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PathVisualizer::publishPath, this));
        
        // Define start and target positions
        float start_x = 10, start_y = 10;
        float target_x = 40, target_y = 40;
        float target_z = 15; // Assuming some fixed height for target

        path_points_ = generatePath(start_x, start_y, target_x, target_y, target_z);
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    vector<pair<float, float>> path_points_;

    vector<pair<float, float>> generatePath(float startX, float startY, float targetX, float targetY, float targetZ) {
        vector<pair<float, float>> path;
        float endLength = 2 * targetZ;
        float theta = atan2(targetY - startY, targetX - startX);
        float stopX = targetX - endLength * cos(theta);
        float stopY = targetY - endLength * sin(theta);
        float dx = stopX - startX;
        float dy = stopY - startY;
        int numOfPointsInLine = 10;
        for (int i = 0; i <= numOfPointsInLine; i++) {
            float px = startX + i * dx / numOfPointsInLine;
            float py = startY + i * dy / numOfPointsInLine;
            path.push_back({px, py});
        }
        return path;
    }

    void publishPath() {
        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.stamp = this->get_clock()->now();
        path_msg.header.frame_id = "map";

        for (const auto& point : path_points_) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = point.first;
            pose.pose.position.y = point.second;
            pose.pose.position.z = 0;
            path_msg.poses.push_back(pose);
        }

        path_pub_->publish(path_msg);

        // Publish the current robot position (start position)
        if (!path_points_.empty()) {
            auto robot_pose = geometry_msgs::msg::PoseStamped();
            robot_pose.header = path_msg.header;
            robot_pose.pose.position.x = path_points_[0].first;
            robot_pose.pose.position.y = path_points_[0].second;
            robot_pose.pose.position.z = 0;
            pose_pub_->publish(robot_pose);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathVisualizer>());
    rclcpp::shutdown();
    return 0;
}
