#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>

#include <ctime>
#include <iomanip>

using namespace std;

float conversionUnit = 1, targetX=0, targetY=0, targetZ = 0.3, currentX = 1, currentY = 49;
vector<pair<float, float>> midpoint;
vector<vector<int>> grid;

// function to create an arbitrary 2D occupancy grid
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
    int n = grid.size(); // Use actual grid size instead of hardcoded value
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

// calculate center of max probability from matrix ie the midpoint
vector<pair<float, float>> calcCoordFromMatrix(const vector<vector<int>>& grid) {
    vector<vector<pair<int, int>>> coordinates = findSquareTargets(grid);
    vector<pair<float, float>> midpoint(1); // Initialize with size 1 // pta nhi kyu ai se fix karaya bug
    
    if (!coordinates.empty()) {
        midpoint[0].first = (coordinates[0][0].first + coordinates[0][2].first) * conversionUnit / 2;
        midpoint[0].second = (coordinates[0][0].second + coordinates[0][2].second) * conversionUnit / 2;
        targetX = midpoint[0].first;
        targetY = midpoint[0].second;
    }
    return midpoint;
}

// a simple path towards the target
vector<pair<float, float>> generatePath(float startX, float startY, float targetX, float targetY) {
    vector<pair<float, float>> path;
    float endLength = 2 * targetZ;
    float thetha = atan((targetY-startY)/(targetX-startX));
    float stopX = targetX - endLength*cos(thetha);
    float stopY = targetY - endLength*sin(thetha);
    float dx = stopX - startX;
    float dy = stopY - startY;
    int numOfPointsInLine = 50;
    for (int i = 0; i <= numOfPointsInLine; i++) {
        float px = startX + i*dx/numOfPointsInLine;
        float py = startY + i*dy/numOfPointsInLine;
        path.push_back({px, py});
    }
    fstream MyFile("points.csv", std::ios::out);
    if (!MyFile) {
        std::cerr << "Error opening file!\n";
    }    
    MyFile << "x" << "," <<"y" << "\n";
    for(int i=0; i < path.size();i++) {
        MyFile << path[i].first << ", " << path[i].second << "\n";
    }
    MyFile.close();
    MyFile.flush();
    return path;
}


int main() {
    grid = createOccupancyGrid(50, 5);
    midpoint = calcCoordFromMatrix(grid);
    
    if (!midpoint.empty()) {
        auto path = generatePath(currentX, currentY, targetX, targetY);
        cout << "Current position: " << currentX << ", " << currentY << "\n"<< "Target's position: " << targetX << ", " << targetY << endl;
        cout << "Path points:" << endl;
        for (int i = 0; i < path.size(); i++) {
           cout << "(" << path[i].first << ", " << path[i].second << ") " << endl;
        }
        cout << "Caculated Stop position: " << path.back().first << ", " << path.back().second << endl;

    } 
    else {
        cout << "No valid targets found!" << endl;
    }
    return 0;
}