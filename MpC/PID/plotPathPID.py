import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

curve_df = pd.read_csv('curve_coordinates.csv')

another_df = pd.read_csv('trajectory.csv')

if 'x' not in another_df.columns or 'y' not in another_df.columns:
    print("Error: 'trajectory.csv' must have 'x' and 'y' columns.")
    
else:
    # Plot both curves on the same graph
    plt.figure(figsize=(8, 5))
    
    plt.plot(curve_df['x'], curve_df['y'], label='Target Path', color='blue', linestyle='--')
    plt.scatter(curve_df['x'], curve_df['y'], color='red', label='Target Path Points')
    
    plt.plot(another_df['x'], another_df['y'], label='Simulated Path', color='green', linestyle='-.')
    plt.scatter(another_df['x'], another_df['y'], color='orange', label='Simulated Path Points')
    
    plt.title('Both Curves in One Graph')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.legend()
    plt.grid(True)
    plt.show()
    
    print("Both curves plotted together successfully.")
