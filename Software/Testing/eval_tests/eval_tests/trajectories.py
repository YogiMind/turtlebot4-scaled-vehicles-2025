import argparse
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import procrustes
from scipy.signal import savgol_filter

# Argument parser
parser = argparse.ArgumentParser(description="Analyze GV vs Robot pose data.")
parser.add_argument("csv_file", help="Path to the CSV file (relative or absolute)")
args = parser.parse_args()

# Resolve to absolute path relative to where the script is run
csv_path = Path(args.csv_file).resolve()

# Load the CSV
data = pd.read_csv(csv_path)

robot_x = data['robot_x']
robot_y = data['robot_y']
gv_x = data['gv_x']
gv_y = data['gv_y']




window_size = 5


gv_x_smooth = pd.Series(data['gv_x']).rolling(window=window_size, center=True).mean()
gv_y_smooth = pd.Series(data['gv_y']).rolling(window=window_size, center=True).mean()


plt.figure(figsize=(17, 10))
plt.plot(gv_x, gv_y, label='GulliView', alpha=0.8)
plt.title('Trajectory GulliView')
plt.show()

plt.figure(figsize=(17, 10))
plt.plot(gv_x_smooth, gv_y_smooth, label='GV_smooth', alpha=0.8)
plt.title('Trajectory GulliView smoothened')
plt.show()

robot_points = np.stack([data['robot_x'], data['robot_y']], axis=1)
gv_points = np.stack([gv_x_smooth, gv_y_smooth], axis=1)

# Subtract centroids
robot_mean = robot_points.mean(axis=0)
gv_mean = gv_points.mean(axis=0)

robot_centered = robot_points - robot_mean
gv_centered = gv_points - gv_mean

# Compute optimal rotation using SVD
U, _, Vt = np.linalg.svd(np.dot(gv_centered.T, robot_centered))
R = np.dot(U, Vt)

# Ensure a proper rotation (det(R) = +1)
if np.linalg.det(R) < 0:
    Vt[-1, :] *= -1
    R = np.dot(U, Vt)

# Compute translation
t = robot_mean - R @ gv_mean

# Transform gv points to robot frame
gv_transformed = (R @ gv_points.T).T + t

# Plot original vs transformed
plt.figure(figsize=(8, 6))
plt.plot(robot_points[:, 0], robot_points[:, 1], 'go-', label='Robot (ground truth)')
plt.plot(gv_points[:, 0], gv_points[:, 1], 'r--', label='GV (raw)')
plt.plot(gv_transformed[:, 0], gv_transformed[:, 1], 'b.-', label='GV → Robot (aligned)')
plt.legend()
plt.title('GV aligned to Robot Coordinates')
plt.axis('equal')
plt.grid(True)
plt.show()






# Prepare input as Nx2 arrays
robot_pts = np.stack([data['robot_x'], data['robot_y']], axis=1)
gv_pts = np.stack([data['gv_x'], data['gv_y']], axis=1)

# Procrustes returns transformed matrices with optimal scaling/rotation/translation
mtx1, mtx2, disparity = procrustes(robot_pts, gv_pts)

plt.figure(figsize=(8, 6))
plt.plot(mtx1[:, 0], mtx1[:, 1], 'go-', label='Robot (ground truth)')
plt.plot(mtx2[:, 0], mtx2[:, 1], 'b.-', label='GV aligned')
plt.title(f'GV aligned to Robot (Procrustes), Disparity: {disparity:.4f}')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.show()
