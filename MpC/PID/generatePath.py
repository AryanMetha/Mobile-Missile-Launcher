import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Generate at least 30 points for the curve
x = np.linspace(0, 20, 60)
amplitude = 3
frequency = 1
y = amplitude * np.sin(frequency * x)

# Save directly to CSV with at least 30 points
df = pd.DataFrame({'x': x, 'y': y})
df.to_csv('curve_coordinates.csv', index=False)

# Plot the curve using matplotlib
plt.figure(figsize=(8, 5))
plt.plot(x, y, label='Sine Wave Curve', color='blue', linestyle='--')
plt.scatter(x, y, color='red', label='Data Points')
plt.title('Curved Road Path')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.legend()
plt.grid(True)
plt.show()

print("CSV file with at least 30 points created and curve plotted successfully.")
