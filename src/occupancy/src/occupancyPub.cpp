#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <vector>
#include <string>
#include <cstdlib>
#include <ctime>
#include <sstream>

using std::vector;

// Helper function to create a 2D occupancy grid
vector<vector<int>> createOccupancyGrid(int size, int submatrixSize) {
    vector<vector<int>> matrix(size, vector<int>(size, 0));
    srand(time(0));
    int maxPosition = size - submatrixSize - 5; // Prevent submatrix at the edge
    int minPosition = 5;                       // Prevent submatrix at the edge
    int startX = (rand() % maxPosition) + minPosition;
    int startY = (rand() % maxPosition) + minPosition;

    for (int i = startX; i < startX + submatrixSize; i++) {
        for (int j = startY; j < startY + submatrixSize; j++) {
            matrix[i][j] = 1;
        }
    }
    return matrix;
}

// Publisher class
class OccupancyMatrixPub : public rclcpp::Node {
public:
    OccupancyMatrixPub() : Node("occupancy_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "topic_for_occupancy_matrix", 10);
        
        // Create a timer that fires every second
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&OccupancyMatrixPub::publish_message, this));
        
        RCLCPP_INFO(this->get_logger(), "Occupancy Matrix Publisher initialized");
    }

private:
    void publish_message() {
        auto message = std_msgs::msg::Int32MultiArray();
        vector<vector<int>> occupancyGrid = createOccupancyGrid(50, 5);

        int rows = occupancyGrid.size();
        int cols = occupancyGrid.empty() ? 0 : occupancyGrid[0].size();

        // Set layout dimensions dynamically
        message.layout.dim.resize(2);
        message.layout.dim[0].label = "rows";
        message.layout.dim[0].size = rows;

        message.layout.dim[1].label = "cols";
        message.layout.dim[1].size = cols;

        // Flatten the 2D vector into a 1D array
        for (const auto &row : occupancyGrid) {
            message.data.insert(message.data.end(), row.begin(), row.end());
        }


        RCLCPP_INFO(this->get_logger(), "Publishing %dx%d array...", rows, cols);
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyMatrixPub>());
    rclcpp::shutdown();
    return 0;
}