#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

struct Point {
    double x, y;
};

int main() {
    std::ifstream file("points.csv");
    if (!file.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return 1;
    }

    std::vector<Point> points;
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        Point p;
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
    
    return 0;
}
