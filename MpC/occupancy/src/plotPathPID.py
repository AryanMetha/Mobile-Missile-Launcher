import matplotlib.pyplot as plt
import pandas as pd

# Function to read CSV files
def read_csv(filename):
    data = pd.read_csv(filename, header=None, names=["x", "y"])
    return data["x"], data["y"]

# Read path and trajectory
path_x, path_y = read_csv("multi_curve_path.csv")
traj_x, traj_y = read_csv("trajectory.csv")

# Plotting
plt.figure(figsize=(10, 6))
plt.plot(path_x, path_y, 'bo-', label="Planned Path", markersize=3)
plt.plot(traj_x, traj_y, 'r-', label="Simulated Trajectory", linewidth=2)

# Mark start and end points
plt.scatter([path_x.iloc[0]], [path_y.iloc[0]], color='green', marker='o', s=100, label="Start Point")
plt.scatter([path_x.iloc[-1]], [path_y.iloc[-1]], color='black', marker='x', s=100, label="End Point")

# Labels and legend
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.title("Car Path Tracking using PID Control")
plt.legend()
plt.grid()

# Show plot
plt.show()
