#include<iostream>
#include<vector>
#include <cmath>

using namespace std;
float conversionUnit, d, x, y;
vector<pair<float,float>> midpoint;


vector<vector<pair<int,int>>> findSquareTargets(const string& grid) {
    int n = 50; // grid is 50x50
    int target_size = 5;
    vector<vector<pair<int, int>>> coordinates;

    for (int row = 0; row <= n - target_size; row++) {
        for (int col = 0; col <= n - target_size; col++) {
            bool found_target = true;
            for (int i = row; i < row + target_size; i++) {
                for (int j = col; j < col + target_size; j++) {
                    if (grid[i * n + j] != '1') {
                        found_target = false;
                        break;
                    }
                }
                if (!found_target) {
                    break;
                }
            }
            if (found_target) {
                vector<pair<int, int>> target;
                target.push_back({row, col});
                target.push_back({row + target_size - 1, col});
                target.push_back({row + target_size - 1, col + target_size - 1});
                target.push_back({row, col + target_size - 1});
                coordinates.push_back(target);
            }
        }
    }
    return coordinates;
}

vector<pair<float,float>> calcCoordFromMatrix() {
    vector<vector<pair<int,int>>> coordinates = findSquareTargets(grid);
    midpoint[0].first = (coordinates[0][0].first + coordinates[0][2].first) / 2;
    midpoint[0].second = (coordinates[0][0].second + coordinates[0][2].second) / 2;
    return midpoint;
}


void forward(float dist) {
    std::cout << "Moving forward" << std::endl;
}

void back() {
    std::cout << "Moving back" << std::endl;
}

void turn(float angle) {
    std::cout << "turning" << std::endl;
}

void initSensors() {
    std::cout << "initalised sensors" << std::endl;
}

void stop() {
    std::cout << "stopped" << std::endl;
}


vector<pair<float,float>> calcXandY(vector<pair<float,float>> midpoint) {   
    vector<pair<float,float>> midpoint;
    midpoint[0].first *= conversionUnit;
    midpoint[0].second *= conversionUnit;
    return midpoint;
}

float calcDistToMove() {
    return sqrt(x*x + pow(d-2y, 2));
}

int calcTurnAngleAlgoA() {
    float thetha = atan(d/x);
    float alpha = atan(x*((2*y)/(d*d +x*x) - (1/d)));
    return (90 - thetha + alpha);
}



int main() {
    initSensors();
    float d = calcPerpDist();
    float x = midpoint[0].first;
    float y = midpoint[0].second;
    midpoint = calcXandY(calcCoordFromMatrix());    
    turn(calcTurnAngleAlgoA());
    forward(calcDistToMove());
    return 0;
}

