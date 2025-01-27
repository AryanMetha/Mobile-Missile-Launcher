#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <vector>
#include <string>
#include <iostream>

using namespace std;

class OccupancyMatrixSub : public rclcpp::Node {
public:
    OccupancyMatrixSub() : Node("occupancy_subscriber") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic_for_occupancy_matrix", 10,
            bind(&OccupancyMatrixSub::topic_callback, this, placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        // received grid data
        RCLCPP_INFO(this->get_logger(), "Received raw occupancy grid:\n%s", msg->data.c_str());

        // remove /n and spaces
        string grid;
        for (char c : msg->data) {
            if (c != ' ' && c != '\n') {
                grid += c;
            }
        }

        // Debug grid
        RCLCPP_INFO(this->get_logger(), "grid size: %zu", grid.size());
        if (grid.size() != 50 * 50) {
            RCLCPP_ERROR(this->get_logger(), "grid size mismatch! Expected 2500, got %zu", grid.size());
            return;
        }

        // find the targets
        vector<pair<int, int>> coordinates = findSquareTargets(grid);

        // results
        if (!coordinates.empty()) {
            RCLCPP_INFO(this->get_logger(), "Found target at:");
            for (const auto& coord : coordinates) {
                RCLCPP_INFO(this->get_logger(), "  (%d, %d)", coord.first, coord.second);
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "no target found");
        }
    }

    vector<pair<int, int>> findSquareTargets(const string& grid) {
        int n = 50; // Assume grid is 50x50
        int target_size = 5;
        vector<pair<int, int>> coordinates;

        for (int row = 0; row <= n - target_size; row++) {
            for (int col = 0; col <= n - target_size; col++) {
                bool found_target = true;

                for (int i = 0; i < target_size; i++) {
                    for (int j = 0; j < target_size; j++) {
                        if (grid[(row + i) * n + (col + j)] != '1') {
                            found_target = false;
                            break;
                        }
                    }
                    if (!found_target) break;
                }

                if (found_target) {
                    coordinates.push_back({row, col});
                }
            }
        }

        return coordinates;
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<OccupancyMatrixSub>());
    rclcpp::shutdown();
    return 0;
}
