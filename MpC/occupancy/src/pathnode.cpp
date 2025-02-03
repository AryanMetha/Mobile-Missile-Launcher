#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <ctime>
//  this is a test node not working right now
using namespace std;

float conversionUnit = 1, targetX = 0, targetY = 0, targetZ = 0.3, currentX = 2, currentY = 3;
vector<pair<float, float>> midpoint;
vector<vector<int>> grid;

// Function to create an arbitrary 2D occupancy grid
vector<vector<int>> createOccupancyGrid(int size, int submatrixSize) {
    vector<vector<int>> matrix(size, vector<int>(size, 0));
    srand(time(0));
    int maxPosition = size - submatrixSize - 5; // prevent submatrix at the edge
    int minPosition = 5;
    
    int startX = (rand() % maxPosition) + minPosition;
    int startY = (rand() % maxPosition) + minPosition;
    
    for (int i = startX; i < startX + submatrixSize; i++) {
        for (int j = startY; j < startY + submatrixSize; j++) {
            matrix[i][j] = 1;
        }
    }
    return matrix;
}

// Find the targets
vector<vector<pair<int, int>>> findSquareTargets(const vector<vector<int>>& grid) {
    int n = grid.size();
    int target_size = 5;
    vector<vector<pair<int, int>>> coordinates;
    
    for (int row = 0; row <= n - target_size; row++) {
        for (int col = 0; col <= n - target_size; col++) {
            bool found_target = true;
            
            for (int i = 0; i < target_size; i++) {
                for (int j = 0; j < target_size; j++) {
                    if (grid[row + i][col + j] != 1) {
                        found_target = false;
                        break;
                    }
                }
                if (!found_target) break;
            }
            
            if (found_target) { // corners
                vector<pair<int, int>> vertex_coords = {
                    {row, col},
                    {row, col + 4},
                    {row + 4, col},
                    {row + 4, col + 4}
                };
                coordinates.push_back(vertex_coords);
            }
        }
    }
    return coordinates;
}

// Calculate center of max probability from matrix ie the midpoint
vector<pair<float, float>> calcCoordFromMatrix(const vector<vector<int>>& grid) {
    vector<vector<pair<int, int>>> coordinates = findSquareTargets(grid);
    vector<pair<float, float>> midpoint(1);
    
    if (!coordinates.empty()) {
        midpoint[0].first = (coordinates[0][0].first + coordinates[0][2].first) * conversionUnit / 2;
        midpoint[0].second = (coordinates[0][0].second + coordinates[0][2].second) * conversionUnit / 2;
        targetX = midpoint[0].first;
        targetY = midpoint[0].second;
    }
    return midpoint;
}    

// Generate a simple path towards the target
vector<pair<float, float>> generatePath(float startX, float startY, float targetX, float targetY) {
    vector<pair<float, float>> path;
    float endLength = 2 * targetZ;
    float thetha = atan((targetY-startY)/(targetX-startX));
    float stopX = targetX - endLength * cos(thetha);
    float stopY = targetY - endLength * sin(thetha);
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

// ROS 2 Node Class
class PathingNode : public rclcpp::Node {
public:
    PathingNode() : Node("pathing_node") {
        // Create a publisher for the path
        path_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("path_points", 10);

        // Create a timer that triggers every second
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&PathingNode::on_timer, this));
    }

private:
    void on_timer() {
        // Create the grid and calculate the path
        grid = createOccupancyGrid(50, 5);
        midpoint = calcCoordFromMatrix(grid);
        
        if (!midpoint.empty()) {
            auto path = generatePath(currentX, currentY, targetX, targetY);

            // Publish the path points
            for (const auto& point : path) {
                geometry_msgs::msg::Point msg;
                msg.x = point.first;
                msg.y = point.second;
                path_publisher_->publish(msg);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "No valid targets found!");
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr path_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathingNode>());
    rclcpp::shutdown();
    return 0;
}
