#!/usr/bin/env python3
"""
ROS2 node to drive the TurtleBot4 in a lawn-mower pattern by publishing cmd_vel,
using odometry feedback for precise 90° turns so SLAM can build an accurate map.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class LawnMowerMapper(Node):
    def __init__(self):
        super().__init__('lawnmower_mapper')
        # Parameters
        self.declare_parameter('width', 8.0)            # X span (m)
        self.declare_parameter('height', 3.7)           # Y span (m)
        self.declare_parameter('spacing', 0.4)          # lane separation (m)
        self.declare_parameter('linear_speed', 0.2)     # m/s
        self.declare_parameter('angular_speed', 0.5)    # rad/s
        self.declare_parameter('cmd_topic', 'cmd_vel')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('turn_tolerance', 0.11)  # rad, compensation for overshoot

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.spacing = self.get_parameter('spacing').value
        self.lin_vel = self.get_parameter('linear_speed').value
        self.ang_vel = self.get_parameter('angular_speed').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.turn_tol = self.get_parameter('turn_tolerance').value

        # Compute number of lanes and shift duration
        self.lanes = math.ceil(self.width / self.spacing)
        self.shift_time = self.spacing / self.lin_vel
        self.forward_time = self.height / self.lin_vel

        # State
        self.current_lane = 0
        self.phase = 0  # 0=forward,1=turn1,2=shift,3=turn2

        # Odometry feedback
        self.current_yaw = 0.0
        self.turn_start_yaw = None
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        # Timer
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.step)
        self.get_logger().info(f"Starting lawn-mower mapping with odom-based turns: {self.lanes} lanes")

    def odom_callback(self, msg):
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def step(self):
        twist = Twist()

        # Completed all lanes
        if self.current_lane >= self.lanes:
            self.cmd_pub.publish(Twist())
            self.get_logger().info('Mapping complete, stopping.')
            rclpy.shutdown()
            return

        # Phase 0: Drive forward along lane
        if self.phase == 0:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            if elapsed < self.forward_time:
                twist.linear.x = self.lin_vel
            else:
                self.phase = 1
                self.turn_start_yaw = None

        # Phase 1: First 90° turn
        elif self.phase == 1:
            if self.turn_start_yaw is None:
                self.turn_start_yaw = self.current_yaw
            # target plus tolerance
            target = (math.pi/2 - self.turn_tol) if self.current_lane % 2 == 0 else (-math.pi/2 + self.turn_tol)
            turned = self.normalize_angle(self.current_yaw - self.turn_start_yaw)
            if abs(turned) < abs(target):
                twist.angular.z = self.ang_vel * (1 if target > 0 else -1)
            else:
                self.phase = 2
                self.start_time = self.get_clock().now()

        # Phase 2: Shift laterally
        elif self.phase == 2:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            if elapsed < self.shift_time:
                twist.linear.x = self.lin_vel
            else:
                self.phase = 3
                self.turn_start_yaw = None

        # Phase 3: Second 90° turn back to forward orientation
        elif self.phase == 3:
            if self.turn_start_yaw is None:
                self.turn_start_yaw = self.current_yaw
            first_target = (math.pi/2 - self.turn_tol) if self.current_lane % 2 == 0 else (-math.pi/2 + self.turn_tol)
            target = -first_target
            turned = self.normalize_angle(self.current_yaw - self.turn_start_yaw)
            if abs(turned) < abs(target):
                twist.angular.z = -self.ang_vel * (1 if target > 0 else -1)
            else:
                self.current_lane += 1
                self.phase = 0
                self.start_time = self.get_clock().now()

        # Publish command
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LawnMowerMapper()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
