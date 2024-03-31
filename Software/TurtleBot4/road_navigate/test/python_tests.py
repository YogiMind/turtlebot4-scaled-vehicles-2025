import numpy as np
import time
import transforms3d as tf
from numba import jit

@jit(nopython=True)  # Apply Numba to grid_transform 
def grid_transform(x_start, x_end, y_start, y_end, cell_size):
    x_values, y_values = np.arange(x_start, x_end + cell_size, cell_size), np.arange(y_start, y_end + cell_size, cell_size)
    X, Y = np.meshgrid(x_values, y_values)
    Z = np.zeros_like(X)
    grid_points = np.column_stack((X, Y, Z))  
    return grid_points

# Convert rotation in degrees around axes to quaternion
def degrees_to_quaternion(rotation_degrees):
    # Convert degrees to radians
    rotation_radians = np.radians(rotation_degrees)
    
    # Apply rotations around each axis
    rotation_quaternion = tf.quaternions.axangle2quat([1, 0, 0], rotation_radians[0])  # Rotation around x-axis
    rotation_quaternion = tf.quaternions.qmult(rotation_quaternion, tf.quaternions.axangle2quat([0, 1, 0], rotation_radians[1]))  # Rotation around y-axis
    rotation_quaternion = tf.quaternions.qmult(rotation_quaternion, tf.quaternions.axangle2quat([0, 0, 1], rotation_radians[2]))  # Rotation around z-axis
    
    return rotation_quaternion

# Example rotation in degrees around axes
rotation_degrees = [90, 0, 0]  # Rotation angles around x, y, z axes respectively

# Convert degrees to quaternion 
rotation_quaternion = degrees_to_quaternion(rotation_degrees)
w, x, y, z = rotation_quaternion

# Pre-compute the rotation matrix once using transforms3d
rotation_matrix = tf.quaternions.quat2mat(rotation_quaternion) 

@jit(nopython=True)  # Apply Numba to transform_points
def transform_points(points, translation):  
    points_rotated = points @ rotation_matrix.T  
    points_transformed = points_rotated + translation
    return points_transformed

# Example usage:
# Define the translation vector
translation = np.array([0, 0, 0])

s = time.time()
points = grid_transform(-1.25, 1.25, 0, 2, 0.01)
transformed_points = transform_points(points, translation)
f = time.time()

print(f - s)  # Print execution time
print(transformed_points)
