#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
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

// Helper function to convert 2D vector to string
std::string convertMatrixToString(const vector<vector<int>> &matrix) {
    std::ostringstream oss;
    for (const auto &row : matrix) {
        for (const auto &elem : row) {
            oss << elem << " ";
        }
        oss << "\n";
    }
    return oss.str();
}

// Publisher class
class OccupancyMatrixPub : public rclcpp::Node {
public:
    OccupancyMatrixPub() : Node("occupancy_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic_for_occupancy_matrix", 10);
        publish_message();
    }

private:
    void publish_message() {
        auto message = std_msgs::msg::String();
        vector<vector<int>> occupancyGrid = createOccupancyGrid(50, 5);
        message.data = convertMatrixToString(occupancyGrid);
        RCLCPP_INFO(this->get_logger(), "Publishing:\n%s", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyMatrixPub>());
    rclcpp::shutdown();
    return 0;
}
