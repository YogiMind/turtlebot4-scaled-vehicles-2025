import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import csv
import math
import os

def get_available_filename(base_name='gv23_lawn_short', extension='.csv'):
    i = 0
    while True:
        filename = f"{base_name}_{i}{extension}"
        if not os.path.exists(filename):
            return filename
        i += 1

class PositionRecorder(Node):

    def __init__(self):
        super().__init__('position_recorder')

        # Define the topics to subscribe to
        self.robot_pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/raphael/pose',  # The robot's SLAM pose topic
            self.robot_pose_callback,
            10
        )
        self.gv_pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/raphael/gv_pose',  # The Gulliview pose topic
            self.gv_pose_callback,
            10
        )

        # Initialize data storage lists
        self.robot_data = []
        self.gv_data = []

        # Open CSV file for writing
        filename = get_available_filename()
        self.csv_file = open(filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'robot_sec', 'robot_nanosec',
            'robot_x', 'robot_y', 'robot_yaw',
            'gv_sec', 'gv_nanosec',
            'gv_x', 'gv_y', 'gv_yaw'
        ])

    def robot_pose_callback(self, msg):
        # Extract timestamp
        stamp = msg.header.stamp

        # Extract robot pose (x, y, yaw)
        robot_x = msg.pose.pose.position.x
        robot_y = msg.pose.pose.position.y
        robot_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        
        # Store robot pose data
        self.robot_data.append((stamp, robot_x, robot_y, robot_yaw))

        # If we have data from both sources, write it to CSV
        if self.gv_data:
            self.write_data()

    def gv_pose_callback(self, msg):
        # Extract timestamp
        stamp = msg.header.stamp

        # Extract Gulliview pose (x, y, yaw)
        gv_x = msg.pose.pose.position.x
        gv_y = msg.pose.pose.position.y
        gv_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)

        # Store Gulliview pose data
        self.gv_data.append((stamp, gv_x, gv_y, gv_yaw))

        # If we have data from both sources, write it to CSV
        if self.robot_data:
            self.write_data()

    def write_data(self):
        if self.robot_data and self.gv_data:
            robot_stamp, robot_x, robot_y, robot_yaw = self.robot_data[-1]
            gv_stamp, gv_x, gv_y, gv_yaw = self.gv_data[-1]
    
            self.csv_writer.writerow([
                robot_stamp.sec, robot_stamp.nanosec,
                robot_x, robot_y, robot_yaw,
                gv_stamp.sec, gv_stamp.nanosec,
                gv_x, gv_y, gv_yaw
            ])

            self.get_logger().info(
                        f"Recorded: robot=({robot_x:.2f}, {robot_y:.2f}, {math.degrees(robot_yaw):.1f}° @ {robot_stamp.sec}.{robot_stamp.nanosec:09d}), "
                        f"gv=({gv_x:.2f}, {gv_y:.2f}, {math.degrees(gv_yaw):.1f}° @ {gv_stamp.sec}.{gv_stamp.nanosec:09d})"
                    )

    def get_yaw_from_quaternion(self, orientation):
        # Convert quaternion to yaw angle (assuming only 2D movement)
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        # Calculate the yaw (rotation around the z-axis)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def __del__(self):
        # Close the CSV file when the node is destroyed
        self.csv_file.close()

def main(args=None):
    rclpy.init(args=args)

    # Create and spin the node
    node = PositionRecorder()
    rclpy.spin(node)

    # Shutdown when done
    rclpy.shutdown()

if __name__ == '__main__':
    main()
