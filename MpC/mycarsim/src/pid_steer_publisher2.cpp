// UNSTABLE



#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>
#include <cmath>
#include <fstream>
#include <chrono>
#include <thread>
#include <sstream>

using namespace std;

double getTargetHeading(const vector<double>& carPos, const vector<double>& targetPos) {
    double dx = targetPos[0] - carPos[0];
    double dy = targetPos[1] - carPos[1];
    return atan2(dy, dx);
}

class PID {
public:
    PID(double kp, double ki, double kd) : Kp(kp), Ki(ki), Kd(kd), prev_error(0), integral(0) {}

    double compute(double error, double dt) {
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        return Kp * error + Ki * integral + Kd * derivative;
    }

private:
    double Kp, Ki, Kd;
    double prev_error, integral;
};

class CarController : public rclcpp::Node {
public:
    CarController()
        : Node("pid_steer_publisher"),
          current_waypoint_index(1),
          speed(1.0),
          dt(0.01),
          car_position({0, 0}),
          heading(getTargetHeading(car, path[currentWaypointIndex])),
          pid(0,0,0) {

        // Initialize publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Load path from CSV
        path = readCSV("/home/shivam/code-masala/pid/src/mycarsim/curve_coordinates.csv");
        if (path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load path!");
            return;
        }

        // Start control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000)),
            std::bind(&CarController::controlLoop, this));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    vector<vector<double>> path;
    vector<double> car_position;
    double heading;
    int current_waypoint_index;
    double speed;
    double dt;

    PID pid;

    // Utility functions
    vector<vector<double>> readCSV(const string &filename) {
        vector<vector<double>> path;
        ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
            return path;
        }

        string line;
        while (getline(file, line)) {
            stringstream ss(line);
            double x, y;
            char comma;
            if (ss >> x >> comma >> y) {
                path.push_back({x, y});
            }
        }
        return path;
    }

    vector<double> subtractVectors(const vector<double> &a, const vector<double> &b) {
        return {a[0] - b[0], a[1] - b[1]};
    }

    double dotProduct(const vector<double> &a, const vector<double> &b) {
        return a[0] * b[0] + a[1] * b[1];
    }

    double cross2D(const vector<double> &v1, const vector<double> &v2) {
        return v1[0] * v2[1] - v1[1] * v2[0];
    }

    vector<double> scaleVector(double scalar, const vector<double> &v) {
        return {scalar * v[0], scalar * v[1]};
    }

    vector<double> getPreviousWaypoint(int index) {
        if (index - 1 < 0) return path[0];
        return path[index - 1];
    }

    bool hasPassedWaypoint(const vector<double> &carPos, const vector<double> &fromPos, const vector<double> &toPos) {
        vector<double> a = subtractVectors(carPos, fromPos);
        vector<double> b = subtractVectors(toPos, fromPos);
        double progress = dotProduct(a, b) / dotProduct(b, b);
        return progress > 1.0;
    }

    double getCrossTrackError(const vector<double> &currentWaypoint, const vector<double> &previousWaypoint, const vector<double> &car) {
        vector<double> v1 = subtractVectors(currentWaypoint, previousWaypoint);
        vector<double> v2 = subtractVectors(car, previousWaypoint);
        vector<double> progress = scaleVector(dotProduct(v1, v2) / dotProduct(v1, v1), v1);
        vector<double> cteVector = subtractVectors(progress, v2);
        return sqrt(dotProduct(cteVector, cteVector));
    }

    int getSteerDirection(const vector<double> &currentWaypoint, const vector<double> &previousWaypoint, const vector<double> &car) {
        vector<double> v1 = subtractVectors(currentWaypoint, previousWaypoint);
        vector<double> v2 = subtractVectors(car, previousWaypoint);
        double crossProduct = cross2D(v2, v1);
        return (crossProduct > 0) ? -1 : 1;
    }



    void controlLoop() {
        if (current_waypoint_index >= path.size()) return;
        
        // Check if waypoint passed
        if (hasPassedWaypoint(car_position, getPreviousWaypoint(current_waypoint_index), path[current_waypoint_index])) {
            current_waypoint_index++;
            heading = getTargetHeading(car, path[currentWaypointIndex]);
        }
        // Compute cross-track error and steering direction
        double error = getCrossTrackError(path[current_waypoint_index], getPreviousWaypoint(current_waypoint_index), car_position);
        error *= getSteerDirection(path[current_waypoint_index], getPreviousWaypoint(current_waypoint_index), car_position);

        // Compute steering command using PID
        double steer = pid.compute(error, dt);

        // Update heading and position
        heading += steer * dt;
        car_position[0] += speed * cos(heading) * dt;
        car_position[1] += speed * sin(heading) * dt;

        //RCLCPP_INFO(this->get_logger(), "Car at (%.2f, %.2f) -> steer: %.2f", car_position[0], car_position[1], steer);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    rclcpp::spin(std::make_shared<CarController>());
    rclcpp::shutdown();
    return 0;
}
