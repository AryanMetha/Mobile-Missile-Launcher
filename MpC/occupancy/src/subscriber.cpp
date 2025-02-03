#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <vector>
#include <iostream>

using namespace std;


vector<vector<int>> convert1DTo2D(const vector<int>& data, int rows, int cols) {
    vector<vector<int>> vec_2d(rows, vector<int>(cols, 0));

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            vec_2d[i][j] = data[i * cols + j];  // Mapping 1D index to 2D
        }
    }
    return vec_2d;
}

class OccupancyMatrixSub : public rclcpp::Node {
public:
    OccupancyMatrixSub() : Node("occupancy_subscriber") {
        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "topic_for_occupancy_matrix", 10,
            bind(&OccupancyMatrixSub::topic_callback, this, placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        // received grid data
        int rows = msg->layout.dim[0].size;
        int cols = msg->layout.dim[1].size;
        RCLCPP_INFO(this->get_logger(), "Received %dx%d Array:", rows, cols);

        // convert 1D to 2D
        vector<vector<int>> grid = convert1DTo2D(msg->data, rows, cols);

        // find the targets
        vector<vector<pair<int, int>>> coordinates = findSquareTargets(grid);

        // results
        if (!coordinates.empty()) {
            RCLCPP_INFO(this->get_logger(), "Found the target with vertices at:");
            for (const auto &coord_set : coordinates) {
                for (const auto &coord : coord_set) {
                    RCLCPP_INFO(this->get_logger(), "  (%d, %d)", coord.first, coord.second);
                }
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Target not found");
        }
    }

    vector<vector<pair<int, int>>> findSquareTargets(const vector<vector<int>>& grid) {
        int n = 50; // grid is 50x50
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

                if (found_target) {
                    // If found, record the four vertices of the 5x5 submatrix
		            vector<pair<int, int>> vertex_coords = {
		                {row, col},                     // Top-left corner
		                {row, col + 4},                 // Top-right corner
		                {row + 4, col},                 // Bottom-left corner
		                {row + 4, col + 4}              // Bottom-right corner
		            };
		            coordinates.push_back(vertex_coords);                }
            }
        }

        return coordinates;
    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<OccupancyMatrixSub>());
    rclcpp::shutdown();
    return 0;
}
