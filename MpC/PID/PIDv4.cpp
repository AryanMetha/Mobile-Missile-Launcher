#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <cmath>
#include <cstdlib>
using namespace std;
int currentWaypointIndex = 1;
double speed = 1.0;
double dt = 0.01;
string input_file = "curve_coordinates.csv";
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

//Get the waypoint before the current waypoint we are driving towards
vector<double> GetPreviousWaypoint(int currentWaypointIndex)
{
    vector<double> previousWaypoint(2, 0);
    if (currentWaypointIndex - 1 < 0)
    {
        previousWaypoint = path[0];
    }
    else
    {
        previousWaypoint = path[currentWaypointIndex - 1];
    }
    return previousWaypoint;
}


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

vector<double> subtractVectors(const vector<double>& a, const vector<double>& b) {
    vector<double> result;
    for (int i = 0; i < a.size(); ++i) {
        result.push_back(a[i] - b[i]);
    }
    return result;
}

double cross2D(vector<double> v1, vector<double>v2){
    return(v1[0]*v2[1] - v1[1]*v2[0]);
}

vector<double> scalerMult(double scalar, const vector<double>& v) {
    vector<double> result;
    for (int i = 0; i < v.size(); ++i) {
        result.push_back(scalar*v[i]);
    }
    return result;
}

double dotProduct(const vector<double>& a, const vector<double>& b) {
    double result = 0 ;
    for (int i = 0; i < a.size(); ++i) {
        result += (a[i] * b[i]);
    }
    return result;
}

double getCrossTrackError(vector<double> currentWaypoint, vector<double> previousWaypoint, vector<double> car)
{
    vector<double> v1 = subtractVectors(currentWaypoint, previousWaypoint);
    vector<double> v2 = subtractVectors(car, previousWaypoint);
    vector<double> progress = scalerMult(dotProduct(v1,v2)/(dotProduct(v1,v1)), v1);
    vector<double> CTE = subtractVectors(progress, v2);
    return sqrt(dotProduct(CTE,CTE));
}

bool hasPassedWaypoint(vector<double> carPos, vector<double> goingFromPos, vector<double> goingToPos)
    {
        bool hasPassedWaypoint = false;

        //The vector between the character and the waypoint we are going from
        vector<double> a = subtractVectors(carPos,goingFromPos);

        //The vector between the waypoints
        vector<double> b = subtractVectors(goingToPos,goingFromPos);

        //Vector projection from https://en.wikipedia.org/wiki/Vector_projection
        //To know if we have passed the upcoming waypoint we need to find out how much of b is a1
        //a1 = (a.b / |b|^2) * b
        //a1 = progress * b -> progress = a1 / b -> progress = (a.b / |b|^2)
        double progress = dotProduct(a,b) / dotProduct(b,b);
        //If progress is above 1 we know we have passed the waypoint
        if (progress > 1.0f)
        {
            hasPassedWaypoint = true;
        }
        return hasPassedWaypoint;
    }

double SteerDirection()
    {
    vector<double> v1 = subtractVectors(path[currentWaypointIndex], GetPreviousWaypoint(currentWaypointIndex));
    vector<double> v2 = subtractVectors(car, GetPreviousWaypoint(currentWaypointIndex));
    double progress_value = dotProduct(v1,v2);
    vector<double> progress = scalerMult(progress_value/(dotProduct(v1,v1)), v1);
    vector<double> CTE = subtractVectors(progress, v2);

    //The cross product between these vectors
    double crossProduct = cross2D(CTE, v1);
    
    // -1 means left
    // 1 means right
    //Now we can decide if we should turn left or right
    int steerDirection = ((crossProduct > 0) ? -1.0 : 1.0);
    return steerDirection;
    }

double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void simulateCar(const vector<vector<double>>& path, const string& output_file, PIDParams params) {
    ofstream output(output_file);
    if (!output.is_open()) {
        cerr << "Error opening output file!" << endl;
        return;
    }
    output << "x,y\n";
   /*  double a,b,c;
    cout<< "Input Kp,Ki,Kd respectively: "<< endl;
    a = 1.0;
    b = 0.001;
    c = 1; */
    // cin >> a >> b >> c;
    PID pid(params.Kp,params.Ki,params.Kd);
    int counter =0;
    //vector<double> car = {path[0][0],path[1][1]};
    while(currentWaypointIndex < path.size()) {
        //cout << "curretn waypoint: " <<path[currentWaypointIndex][0] <<" " <<path[currentWaypointIndex][1]<< endl;
        //cout << "prev  waypoint: " << GetPreviousWaypoint(currentWaypointIndex)[0] << " " << GetPreviousWaypoint(currentWaypointIndex)[1] << endl;
        double error = getCrossTrackError(path[currentWaypointIndex], GetPreviousWaypoint(currentWaypointIndex), car);
        //cout << "error: " << error << endl;
        error *= SteerDirection();
        double steer = (pid.compute(error, dt));
        static double heading = 0;  // Maintain heading over time
        heading += steer * dt;  // Adjust heading based on steering correction
        car[0] += speed * cos(heading) * dt;
        car[1] += speed * sin(heading) * dt;

        output << car[0] << "," << car[1] << "\n";  // Write trajectory
        // output << counter <<"," << error << "\n";  // Write eror
        //cout << "car coordinates: " << car[0] << "," << car[1] << "\n";  // Write trajectory
        // cout  << endl;
        counter++;
        if (hasPassedWaypoint(car,GetPreviousWaypoint(currentWaypointIndex),path[currentWaypointIndex])) currentWaypointIndex++;
    }
    cout<< "counter: " << counter << endl;
    output.close();
}



double runPIDTest(auto path, PIDParams params) {
    PID pid(params.Kp,params.Ki,params.Kd);
    int currentWaypointIndex = 1;
    int counter =0;
    double totalError =0;
    vector<double> car = {0,0};
    while(currentWaypointIndex < path.size()) {
        // cout << "curretn waypoint: " <<path[currentWaypointIndex][0] <<" " <<path[currentWaypointIndex][1]<< endl;
        //cout << "prev  waypoint: " << GetPreviousWaypoint(currentWaypointIndex)[0] << " " << GetPreviousWaypoint(currentWaypointIndex)[1] << endl;
        double error = getCrossTrackError(path[currentWaypointIndex], GetPreviousWaypoint(currentWaypointIndex), car);
        // cout << "error: " << error << endl;
        error *= SteerDirection();
        double steer = (pid.compute(error, dt));
        static double heading = 0;  // Maintain heading over time
        heading += steer * dt;  // Adjust heading based on steering correction
        car[0] += speed * cos(heading) * dt;
        car[1] += speed * sin(heading) * dt;
        //cout << "car coordinates: " << car[0] << "," << car[1] << "\n";  // Write trajectory
        // cout  << endl;
        totalError += error*error;
        counter++;
        if (counter > 5000) {break;}
        if (hasPassedWaypoint(car,GetPreviousWaypoint(currentWaypointIndex),path[currentWaypointIndex])) currentWaypointIndex++;
    }
    return totalError;
}
void tunePID(const vector<vector<double>>& path) {
    PIDParams bestParams;
    double minError = numeric_limits<double>::max();
    
    for (double Kp = 0.1; Kp <= 2.0; Kp += 0.1) {
        for (double Ki = 0.0; Ki <= 1.0; Ki += 0.1) {
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
    }
    cout << "best params" << bestParams.Kp << " " << bestParams.Ki << " " << bestParams.Kd << endl;
    simulateCar(path, output_file, bestParams);

}

int main() {
    path = readCSV(input_file);
    if (path.empty()) {
        cerr << "Error reading path data!" << endl;
        return 1;
    }

    //simulateCar(path, output_file);
    tunePID(path);
    cout << "Simulation completed. Trajectory saved in " << output_file << endl;
    system("python3 plotPathPID.py");
    return 0;
}