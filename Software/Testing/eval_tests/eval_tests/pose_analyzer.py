# pip install pandas numpy scikit-learn seaborn matplotlib

import argparse
from pathlib import Path
import pandas as pd
import numpy as np
from sklearn.linear_model import LinearRegression
import matplotlib.pyplot as plt
from scipy.linalg import svd

# Corners:
# (692, 178)
# (9823, 164)
# (352, 5071)
# (9478, 4778)



# Argument parser
parser = argparse.ArgumentParser(description="Analyze GV vs Robot pose data.")
parser.add_argument("csv_file", help="Path to the CSV file (relative or absolute)")
args = parser.parse_args()

# Resolve to absolute path relative to where the script is run
csv_path = Path(args.csv_file).resolve()

# Load the CSV
data = pd.read_csv(csv_path)

# Convert to full nanosecond timestamps
data['robot_time_ns'] = data['robot_sec'] * 1e9 + data['robot_nanosec']
data['gv_time_ns'] = data['gv_sec'] * 1e9 + data['gv_nanosec']

# Create separate DataFrames for GV and robot
gv_df = data[['gv_time_ns', 'gv_x', 'gv_y', 'gv_yaw']].copy()
robot_df = data[['robot_time_ns', 'robot_x', 'robot_y', 'robot_yaw']].copy()

# Sort both by time
gv_df = gv_df.sort_values('gv_time_ns')
robot_df = robot_df.sort_values('robot_time_ns')

# Merge asof: For each GV reading, find the closest robot reading before or after
merged = pd.merge_asof(
    gv_df, 
    robot_df, 
    left_on='gv_time_ns', 
    right_on='robot_time_ns',
    direction='nearest',  # closest in time (forward or backward)
    tolerance=100_000_000  # 100 ms threshold
)

# Drop rows where no match was found
merged = merged.dropna()

# Now do the regression analysis
gv_x = merged['gv_x'].values.reshape(-1, 1)
gv_y = merged['gv_y'].values.reshape(-1, 1)
robot_x = merged['robot_x'].values
robot_y = merged['robot_y'].values



# Fit linear models
model_x = LinearRegression().fit(gv_x, robot_x)
model_y = LinearRegression().fit(gv_y, robot_y)

# Coefficients
a_x, b_x = model_x.coef_[0], model_x.intercept_
a_y, b_y = model_y.coef_[0], model_y.intercept_

print(f"Linear Approximation for X: a_x = {a_x:.4f}, b_x = {b_x:.4f}")
print(f"Linear Approximation for Y: a_y = {a_y:.4f}, b_y = {b_y:.4f}")


#-----------------------------------------------------


# Get starting positions
gv_start = np.array([gv_df['gv_x'].iloc[0], gv_df['gv_y'].iloc[0]])
slam_start = np.array([robot_df['robot_x'].iloc[0], robot_df['robot_y'].iloc[0]])


# Scale and align
robot_x_scaled = merged['robot_x'] / a_x
robot_y_scaled = merged['robot_y'] / a_y

# Compute offset to align start positions
aligned_start = gv_start - np.array([robot_x_scaled.iloc[0], robot_y_scaled.iloc[0]])

# Apply offset
robot_x_aligned = robot_x_scaled + aligned_start[0]
robot_y_aligned = robot_y_scaled + aligned_start[1]



plt.figure(figsize=(17, 10))

# Original GV path
plt.plot(merged['gv_x'], merged['gv_y'], label='Gulliview (GV)', color='red', linewidth=2)

# Transformed SLAM path
plt.plot(robot_x_aligned, robot_y_aligned, label='SLAM (aligned to GV)', color='blue', linewidth=2, linestyle='--')

plt.xlabel('X')
plt.ylabel('Y')
plt.title("Trajectory Comparison (SLAM aligned to GV)")
plt.legend()
plt.gca().set_aspect('equal', adjustable='box')
plt.grid(True)
plt.show()


#-----------------------------------------------------


# Create point arrays (N x 2)
gv_points = merged[['gv_x', 'gv_y']].to_numpy()
robot_points = merged[['robot_x', 'robot_y']].to_numpy()

gv_start = gv_points[0]
robot_start = robot_points[0]
gv_centered = gv_points - gv_start
robot_centered = robot_points - robot_start

# SVD to solve for optimal rotation/scale
U, S, Vt = svd(gv_centered.T @ robot_centered)
R = (U @ Vt).T  # 2x2 rotation matrix
scale = np.trace(np.diag(S)) / np.sum(robot_centered ** 2)

# Apply transformation: rotate + scale + translate
robot_transformed = scale * (robot_centered @ R) + gv_start


plt.figure(figsize=(17, 10))
plt.plot(gv_points[:, 0], gv_points[:, 1], label='GulliView (GV)', color='red', linewidth=2)
plt.plot(robot_transformed[:, 0], robot_transformed[:, 1], label='SLAM (transformed)', color='blue', linewidth=2, linestyle='--')

plt.xlabel('X')
plt.ylabel('Y')
plt.title("Trajectory Comparison with Rotation, Scale, and Translation")
plt.legend()
plt.gca().set_aspect('equal', adjustable='box')
plt.grid(True)
plt.show()


#-----------------------------------------------------

# Predict and compute errors
estimated_x = model_x.predict(gv_x)
estimated_y = model_y.predict(gv_y)
error_x = robot_x - estimated_x
error_y = robot_y - estimated_y
error = np.sqrt(error_x**2 + error_y**2)
merged['error'] = error

plt.figure(figsize=(17, 10))
plt.plot(merged['robot_x'], merged['robot_y'], label='SLAM (Robot)', alpha=0.8)
plt.plot(merged['gv_x'], merged['gv_y'], label='Gulliview (GV)', alpha=0.8)
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.gca().set_aspect('equal', adjustable='box')
plt.title("Trajectory Comparison")
plt.grid(True)
plt.show()

# Scatter error plot
plt.figure(figsize=(17, 10))
plt.scatter(merged['gv_x'], merged['gv_y'], c=error, cmap='coolwarm', s=40)
plt.colorbar(label='Error')
plt.xlabel('GV X')
plt.ylabel('GV Y')
plt.title('GV→Robot Position Error')

plt.xlim(100, 10000)
plt.ylim(100, 6000)
plt.gca().set_aspect('equal', adjustable='box')
plt.grid(True)
plt.show()

# R² scores
r2_x = model_x.score(gv_x, robot_x)
r2_y = model_y.score(gv_y, robot_y)
print(f"R² for X: {r2_x:.4f}")
print(f"R² for Y: {r2_y:.4f}")
