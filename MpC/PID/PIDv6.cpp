#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <cmath>
#include <cstdlib>
using namespace std;

int currentWaypointIndex = 1;
double speed = 1;
double angular_speed = 0.1;
double dt = 0.04;
string input_file = "sine.csv";
string output_file = "trajectory.csv";
vector<vector<double>> path;
vector<double> car = {0, 0};

struct PIDParams {
    double Kp, Ki, Kd;
};

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

// Utility functions
vector<double> subtractVectors(const vector<double>& a, const vector<double>& b) {
    vector<double> result(a.size());
    for (int i = 0; i < a.size(); ++i) {
        result[i] = a[i] - b[i];
    }
    return result;
}

vector<double> scaleVector(double scalar, const vector<double>& v) {
    vector<double> result(v.size());
    for (int i = 0; i < v.size(); ++i) {
        result[i] = scalar * v[i];
    }
    return result;
}

double dotProduct(const vector<double>& a, const vector<double>& b) {
    double result = 0;
    for (int i = 0; i < a.size(); ++i) {
        result += a[i] * b[i];
    }
    return result;
}

double cross2D(const vector<double>& v1, const vector<double>& v2) {
    return v1[0] * v2[1] - v1[1] * v2[0];
}

double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// File reading
vector<vector<double>> readCSV(const string& filename) {
    vector<vector<double>> path;
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error opening file!" << endl;
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

// Waypoint utilities
vector<double> getPreviousWaypoint(int index) {
    if (index - 1 < 0) {
        return path[0];
    }
    return path[index - 1];
}

bool hasPassedWaypoint(const vector<double>& carPos, const vector<double>& fromPos, const vector<double>& toPos) {
    vector<double> a = subtractVectors(carPos, fromPos);
    vector<double> b = subtractVectors(toPos, fromPos);
    double progress = dotProduct(a, b) / dotProduct(b, b);
    return progress > 1.0;
}

// Error calculation
double getCrossTrackError(const vector<double>& currentWaypoint, const vector<double>& previousWaypoint, const vector<double>& car) {
    vector<double> v1 = subtractVectors(currentWaypoint, previousWaypoint);
    vector<double> v2 = subtractVectors(car, previousWaypoint);
    vector<double> progress = scaleVector(dotProduct(v1, v2) / dotProduct(v1, v1), v1);
    vector<double> cteVector = subtractVectors(progress, v2);
    return sqrt(dotProduct(cteVector, cteVector));
}

int getSteerDirection(const vector<double>& currentWaypoint, const vector<double>& previousWaypoint, const vector<double>& car) {
    vector<double> v1 = subtractVectors(currentWaypoint, previousWaypoint);
    vector<double> v2 = subtractVectors(car, previousWaypoint);
    vector<double> progress = scaleVector(dotProduct(v1, v2) / dotProduct(v1, v1), v1);
    vector<double> cteVector = subtractVectors(progress, v2);
    double crossProduct = cross2D(cteVector, v1);
    return (crossProduct > 0) ? -1 : 1;
}

double getTargetHeading(const vector<double>& carPos, const vector<double>& targetPos) {
    double dx = targetPos[0] - carPos[0];
    double dy = targetPos[1] - carPos[1];
    return atan2(dy, dx);
}


// Simulation
void simulateCar(const vector<vector<double>>& path, const string& output_file, const PIDParams& params) {
    ofstream output(output_file);
    if (!output.is_open()) {
        cerr << "Error opening output file!" << endl;
        return;
    }

    output << "x,y\n";
    PID pid(params.Kp, params.Ki, params.Kd);
    int counter = 0;
    double heading = getTargetHeading(car, path[currentWaypointIndex]);
    while (currentWaypointIndex +1< path.size()) {
        if (hasPassedWaypoint(car, getPreviousWaypoint(currentWaypointIndex), path[currentWaypointIndex])) {
            currentWaypointIndex++;
            cout << "Reached the next waypoint" << endl;
            heading = getTargetHeading(car, path[currentWaypointIndex]);
        }
        // if (currentWaypointIndex = path.size()-1) break;
        double error = getCrossTrackError(path[currentWaypointIndex], getPreviousWaypoint(currentWaypointIndex), car);
        error *= getSteerDirection(path[currentWaypointIndex], getPreviousWaypoint(currentWaypointIndex), car);
        double steer = pid.compute(error, dt);
        double thetha =  steer * dt;
        double t = thetha/angular_speed;
        // non ideal twisting time
        if (heading == heading+thetha) {
            car[0] += speed * cos(heading) * dt;
            car[1] += speed * sin(heading) * dt;
        }
        while(heading != heading+thetha) {
            heading += thetha*dt;
            cout << "heading: " << heading << endl;
            car[0] += speed * cos(heading) * dt;
            car[1] += speed * sin(heading) * dt;
            cout << "next waypoint: " <<path[currentWaypointIndex][0] <<" " <<path[currentWaypointIndex][1]<< endl;
            cout << "car coordinates: " << car[0] << "," << car[1] << "\n";  // Write trajectory
            cout << endl;
            output << car[0] << "," << car[1] << "\n";
        }
        cout << "heading: " << heading << ", counter: " << counter<< endl;
        counter++;
    }

    cout << "Counter: " << counter << endl;
    output.close();
}

// PID testing and tuning
/* double runPIDTest(const vector<vector<double>>& path, const PIDParams& params) {
    PID pid(params.Kp, params.Ki, params.Kd);
    int currentWaypointIndex = 1;
    int counter = 0;
    double totalError = 0;
    vector<double> car = {0, 0};
    double heading = getTargetHeading(car, path[currentWaypointIndex]);

    while (currentWaypointIndex < path.size()) {
        double error = getCrossTrackError(path[currentWaypointIndex], getPreviousWaypoint(currentWaypointIndex), car);
        error *= getSteerDirection(path[currentWaypointIndex], getPreviousWaypoint(currentWaypointIndex), car);
        double steer = pid.compute(error, dt);

        heading += steer * dt;
        car[0] += speed * cos(heading) * dt;
        car[1] += speed * sin(heading) * dt;
        totalError += (error)*(error);
        counter++;

        if (counter > 5000) break;
        if (hasPassedWaypoint(car, getPreviousWaypoint(currentWaypointIndex), path[currentWaypointIndex])) {
            currentWaypointIndex++;
            heading = getTargetHeading(car, path[currentWaypointIndex]);
        }
    }
    return totalError;
} */

void tunePID(const vector<vector<double>>& path) {
    PIDParams bestParams;
    double minError = numeric_limits<double>::max();

       /*  for (double Kp = 0.1; Kp <= 3.0; Kp += 0.1) {
            for (double Ki = 0.0; Ki <= 0.5; Ki += 0.1) {
                for (double Kd = 0.1; Kd <= 2.0; Kd += 0.1) {
                    double error = runPIDTest(path, {Kp, Ki, Kd});
                    if (error < minError) {
                        minError = error;
                        bestParams = {Kp, Ki, Kd};
                    }
                    cout << "total error: " << error << endl;
                    cout << "params: " << Kp << " " << Ki << " " << Kd << endl;
                    cout << endl;
                }
            }
        } */

    bestParams = {0,0,0};

    cout << "params: " << bestParams.Kp << " " << bestParams.Ki << " " << bestParams.Kd << endl;
    simulateCar(path, output_file, bestParams);
}

int main() {
    path = readCSV(input_file);
    if (path.empty()) return 1;

    tunePID(path);
    cout << "Simulation completed. Trajectory saved in " << output_file << endl;
    system("python3 plotPathPID.py");

    return 0;
}
