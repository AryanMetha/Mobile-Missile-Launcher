#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

double Kp = 0.1;
double target_speed = 5.0; 
double actual_speed = 0.0; 
double dt = 0.1; 

struct Pose {
    double x, y,angle;
};

std::vector<Pose> readCSV() {
    std::ifstream file("points.csv");
    if (!file.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        }

    std::vector<Pose> points;
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        Pose p;
        char comma;
        if (ss >> p.x >> comma >> p.y) {
            points.push_back(p);
        } else {
            std::cerr << "Error reading line: " << line << std::endl;
        }
    }

    file.close();

    std::cout << "Read Points:" << std::endl;
    for (const auto& p : points) {
        std::cout << "(" << p.x << ", " << p.y << ")" << std::endl;
    }
    return points;
}

double compute(double setpoint, double actual) {
        double error = setpoint - actual;
        return Kp * error;
}

int main() {
    std::vector<Pose> required_path = readCSV();
    Pose current_pose = {required_path[0].x, required_path[0].y, required_path[0].angle}; 
    std::vector<Pose> simulated_path = {current_pose};


    for (int i = 1; i < required_path.size(); i++) {
        Pose next_pose = required_path[i];
        while((current_pose.x != next_pose.x)&&(current_pose.y != next_pose.y)&&(current_pose.angle != next_pose.angle) ) {
            
        }





        }


    





    return 0;
}
