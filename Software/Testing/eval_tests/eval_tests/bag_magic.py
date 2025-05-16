import argparse
from pathlib import Path
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseWithCovarianceStamped
import matplotlib.pyplot as plt
import math
import numpy as np
from scipy.spatial import procrustes
from scipy.linalg import svd

# === Your exact yaw function reused ===
def get_yaw_from_quaternion(orientation):
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def get_msg_time(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

## === Read bag file ===
#def read_pose_data(bag_path, topics):
#    reader = SequentialReader()
#    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
#    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
#    reader.open(storage_options, converter_options)
#
#    all_topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
#    data = {topic: [] for topic in topics}
#
#    type_map = {
#        'geometry_msgs/msg/PoseWithCovarianceStamped': PoseWithCovarianceStamped
#    }
#
#    while reader.has_next():
#        topic, data, _ = reader.read_next()
#        if topic not in topics:
#            continue
#        msg_type = type_map[all_topics[topic]]
#        msg = deserialize_message(data, msg_type)
#        data[topic].append(msg)
#
#    return data



def read_pose_data(bag_path, topics):
    reader = SequentialReader()
    storage_options = StorageOptions(uri=str(bag_path), storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader.open(storage_options, converter_options)

    all_topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    data = {topic: [] for topic in topics}

    type_map = {
        'geometry_msgs/msg/PoseWithCovarianceStamped': PoseWithCovarianceStamped
    }

    while reader.has_next():
        topic, data_bytes, _ = reader.read_next()  # Corrected variable names
        if topic not in topics:
            continue
        msg_type = type_map[all_topics[topic]]
        msg = deserialize_message(data_bytes, msg_type)
        data[topic].append(msg)  # Store deserialized messages in dictionary

    return data


# === Sync robot and GV pose by timestamp ===
def sync_by_timestamp(robot_msgs, gv_msgs, threshold=0.10):
    i, j = 0, 0
    synced = []
    while i < len(robot_msgs) and j < len(gv_msgs):
        t1 = get_msg_time(robot_msgs[i])
        t2 = get_msg_time(gv_msgs[j])
        if abs(t1 - t2) <= threshold:
            synced.append((robot_msgs[i], gv_msgs[j]))
            i += 1
            j += 1
        elif t1 < t2:
            i += 1
        else:
            j += 1
    return synced

def find_first_x_exceeds(data, m):
    for i, (x, y) in enumerate(data):
        if x > m or y > m:
            return i
    return None  # No match found


def translate_to_origin(points):
    x0, y0 = points[0]
    return [(x - x0, y - y0) for x, y in points]


def align_trajectories(robot_trajectory, gv_trajectory):
    # Convert trajectories to numpy arrays
    robot_array = np.array(robot_trajectory)
    gv_array = np.array(gv_trajectory)

    # Apply Procrustes Analysis for best alignment
    _, aligned_gv, transformation = procrustes(robot_array, gv_array)

    # Return the aligned GV trajectory
    return aligned_gv


# === Example usage ===
if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Analyze GV vs Robot pose data.")
    parser.add_argument("bag_folder", help="Path to the rosbag folder (relative or absolute)")
    args = parser.parse_args()
    
    # Resolve to absolute path relative to where the script is run
    bag_path = Path(args.bag_folder).resolve()


    topics = ['/raphael/pose', '/raphael/gv_pose']
    data = read_pose_data(bag_path, topics)

    robot_msgs = data['/raphael/pose']
    gv_msgs = data['/raphael/gv_pose']

    robot_points = np.array([(msg.pose.pose.position.x, msg.pose.pose.position.y) for msg in robot_msgs])
    gv_points    = np.array([(msg.pose.pose.position.x / 1000.0, msg.pose.pose.position.y / 1000.0) for msg in gv_msgs])

    plt.figure(figsize=(17, 10))
    plt.plot(gv_points[:, 0], gv_points[:, 1], label=f'GulliView {len(gv_points)}', color='red', linewidth=2)
    plt.plot(robot_points[:, 0], robot_points[:, 1], label=f'SLAM {len(robot_points)}', color='blue', linewidth=2)
    
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title("Trajectory Comparison RAW")
    plt.legend()
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True)
    plt.show()

    synced = sync_by_timestamp(robot_msgs, gv_msgs)

    robot_points = np.array([(msg.pose.pose.position.x, msg.pose.pose.position.y) for msg, _ in synced])
    gv_points    = np.array([(msg.pose.pose.position.x / 1000.0, msg.pose.pose.position.y / 1000.0) for _, msg in synced])
    cut = find_first_x_exceeds(gv_points, 50000)


    if cut:
        gv_points = gv_points[:cut]
        robot_points = robot_points[:cut]
 
    plt.figure(figsize=(17, 10))
    plt.plot(gv_points[:, 0], gv_points[:, 1], label=f'GulliView {len(gv_points)}', color='red', linewidth=2)
    plt.plot(robot_points[:, 0], robot_points[:, 1], label=f'SLAM {len(robot_points)}', color='blue', linewidth=2)
    
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title("Trajectory Comparison synced")
    plt.legend()
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True)
    plt.show()

    
    gv_start = gv_points[0]
    robot_start = robot_points[0]
    gv_centered = gv_points - gv_start
    robot_centered = robot_points - robot_start
    
    # SVD to solve for optimal rotation
    U, S, Vt = svd(gv_centered.T @ robot_centered)
    R = (U @ Vt).T  # 2x2 rotation matrix
    
    robot_transformed = (robot_centered @ R) + gv_start
    
    
    plt.figure(figsize=(17, 10))
    plt.plot(gv_points[:, 0], gv_points[:, 1], label=f'GulliView {len(gv_points)}', color='red', linewidth=2)
    plt.plot(robot_transformed[:, 0], robot_transformed[:, 1], label=f'SLAM {len(robot_transformed)}', color='blue', linewidth=2)
    
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title("Trajectory Comparison with Rotation and Translation")
    plt.legend()
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True)
    plt.show()

