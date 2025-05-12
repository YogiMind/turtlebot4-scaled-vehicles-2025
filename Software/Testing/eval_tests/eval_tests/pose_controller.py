# go_to_pose.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
import math

max_linear_speed = 0.15
min_linear_speed = 0.02
max_angular_speed = 1.0
min_angular_speed = 0.05

def quaternion_to_yaw(q):
    # Converts quaternion to yaw (Z axis rotation)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class GoToPose(Node):
    def __init__(self):
        super().__init__('go_to_pose')
        
        # List of predefined positions (x, y, yaw in degrees)
        self.targets = [
            (8729, 3184, 90.0),
            (1.0, 0.0, 90.0),
            (1.0, 1.0, 180.0),
            (0.0, 1.0, -90.0),
        ]

        self.declare_parameter('target_id', 0)
        target_id = self.get_parameter('target_id').get_parameter_value().integer_value


        if 0 <= target_id < len(self.targets):
            x, y, yaw_deg = self.targets[target_id]
            self.target_x = x
            self.target_y = y
            self.target_yaw = math.radians(yaw_deg)
            self.get_logger().info(f"Going to target {target_id}: ({x}, {y}, {yaw_deg}°)")
        else:
            self.get_logger().error("Invalid target_id parameter!")
            rclpy.shutdown()
            return

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/raphael/gv_pose',
            self.pose_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/raphael/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.latest_pose = None

    def pose_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(ori)
        self.latest_pose = (pos.x, pos.y, yaw)

    def control_loop(self):
        if self.latest_pose is None:
            return
    
        x, y, yaw = self.latest_pose
    
        dx = self.target_x - x
        dy = self.target_y - y
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        angle_error = math.atan2(math.sin(angle_to_target - yaw), math.cos(angle_to_target - yaw))
    
        cmd = Twist()
    
        # Normalize yaw error to target orientation
        yaw_error = math.atan2(math.sin(self.target_yaw - yaw), math.cos(self.target_yaw - yaw))
    
        # --- Case 1: Positioning ---
        if distance > 0.03:
            # Smooth speed near goal
            linear_speed = max(min_linear_speed, min(max_linear_speed, distance * 0.5))
            angular_speed = max(min_angular_speed, min(max_angular_speed, abs(angle_error) * 1.5))
    
            if abs(angle_error) > 0.2:
                cmd.angular.z = angular_speed * math.copysign(1, angle_error)
                cmd.linear.x = 0.0
            else:
                cmd.linear.x = linear_speed
                cmd.angular.z = angular_speed * math.copysign(1, angle_error)
    
        # --- Case 2: At position, final alignment ---
        elif abs(yaw_error) > 0.05:
            angular_speed = max(min_angular_speed, min(max_angular_speed, abs(yaw_error) * 1.5))
            cmd.angular.z = angular_speed * math.copysign(1, yaw_error)
            cmd.linear.x = 0.0
    
        # --- Case 3: Done ---
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info("Target reached!")
    
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = GoToPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
