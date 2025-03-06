#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

struct Point {
    double x, y;
};

//  CSV Reader Function (with logging)
std::vector<Point> readCSV(const std::string& filename) {
    std::vector<Point> points;
    std::ifstream file(filename);
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        double x, y;
        char comma;
        if (ss >> x >> comma >> y) {
            points.push_back({x, y});
            RCLCPP_INFO(rclcpp::get_logger("readCSV"), "Read point: (%.2f, %.2f)", x, y);
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("readCSV"), 
                "Total points loaded from %s: %zu", filename.c_str(), points.size());
    return points;
}

//  Path Marker (LINE_STRIP)
visualization_msgs::msg::Marker createPathMarker(
    const std::vector<Point>& points,
    const std::string& frame_id,
    int id,
    float r, float g, float b) 
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "paths";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.01; // Thin line
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    marker.lifetime = rclcpp::Duration::from_seconds(0);  // Static forever
    marker.frame_locked = true;

    for (const auto& point : points) {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0.0;
        marker.points.push_back(p);
    }
    return marker;
}

visualization_msgs::msg::Marker createPointListMarker(
    const std::vector<Point>& points,
    const std::string& frame_id,
    int id,
    float r, float g, float b) 
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "points";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.15; // Sphere size
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    marker.lifetime = rclcpp::Duration::from_seconds(0); // Static forever
    marker.frame_locked = true;

    for (const auto& point : points) {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0.0;
        marker.points.push_back(p);
    }

    return marker;
}

class PathPublisher : public rclcpp::Node {
public:
    PathPublisher() : Node("path_publisher") {
        //  Get CSV Paths Using ament_index_cpp
        std::string target_csv = ament_index_cpp::get_package_share_directory("car_path_simulation") + "/share/csv/curve_coordinates.csv";
        std::string simulated_csv = ament_index_cpp::get_package_share_directory("car_path_simulation") + "/share/csv/trajectory.csv";
        
        //  Read Points from CSV Files
        target_points_ = readCSV(target_csv);
        simulated_points_ = readCSV(simulated_csv);

        //  Create Publishers
        target_path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("target_path", 10);
        simulated_path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("simulated_path", 10);

        //  Publish Markers Periodically
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),  // Publish every 500ms (2Hz)
            std::bind(&PathPublisher::publishAllMarkers, this)
        );
    }

private:
    void publishAllMarkers() {
        //  Publish Target Path (Green)
        auto target_marker = createPathMarker(target_points_, "map", 0, 0.0, 1.0, 0.0);
        target_path_pub_->publish(target_marker);

        //  Publish Simulated Path (Red)
        auto simulated_marker = createPathMarker(simulated_points_, "map", 1, 1.0, 0.0, 0.0);
        simulated_path_pub_->publish(simulated_marker);

        //  Publish Start and End Points (Yellow)
        std::vector<Point> special_points;
        if (!target_points_.empty()) {
            special_points.push_back(target_points_.front()); // Target start
            special_points.push_back(target_points_.back());  // Target end
        }
        if (!simulated_points_.empty()) {
            special_points.push_back(simulated_points_.front()); // Sim start
            special_points.push_back(simulated_points_.back());  // Sim end
        }

        auto point_marker = createPointListMarker(special_points, "map", 2, 1.0, 1.0, 0.0);
        target_path_pub_->publish(point_marker);

/*         RCLCPP_INFO(this->get_logger(), "Markers published.");
 */    }

    //  Publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr simulated_path_pub_;

    //  Timer for periodic publishing
    rclcpp::TimerBase::SharedPtr timer_;

    //  Point Containers
    std::vector<Point> target_points_;
    std::vector<Point> simulated_points_;
};

//  Main Function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPublisher>());
    rclcpp::shutdown();
    return 0;
}
