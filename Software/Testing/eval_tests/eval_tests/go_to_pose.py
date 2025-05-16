import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import threading

max_linear_speed = 0.15
min_linear_speed = 0.02
max_angular_speed = 1.0

namespace = "/glenn"

class GoToXY(Node):
    def __init__(self):
        super().__init__('go_to_xy')
        
        self.targets = [
            (8144, 1573),
            (1632, 1536),
            (1727, 4465),
            (8194, 4469),
        ]

        self.current_pose = None
        self.prev_pose = None
        self.estimated_yaw = 0.0
        self.target_index = 0
        self.target = self.targets[self.target_index]

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            namespace + '/gv_pose',
            self.pose_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, namespace + '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        # Start input thread
        threading.Thread(target=self.input_thread, daemon=True).start()

    def input_thread(self):
        while rclpy.ok():
            try:
                new_index = int(input("Enter target index (0–{}): ".format(len(self.targets)-1)))
                if 0 <= new_index < len(self.targets):
                    self.target_index = new_index
                    self.target = self.targets[self.target_index]
                    self.get_logger().info(f"Switching to target {self.target_index}: {self.target}")
                else:
                    print("Invalid index.")
            except Exception as e:
                print(f"Input error: {e}")

    def pose_callback(self, msg):
        pos = msg.pose.pose.position
        self.current_pose = (pos.x, pos.y)

    def control_loop(self):
        if self.current_pose is None:
            return

        x, y = self.current_pose
        gx, gy = self.target

        # Estimate yaw from motion
        if self.prev_pose is not None:
            px, py = self.prev_pose
            dx = x - px
            dy = y - py
            if math.hypot(dx, dy) > 0.001:
                self.estimated_yaw = math.atan2(dy, dx)

        self.prev_pose = (x, y)

        # Compute distance and vector to goal
        dx = gx - x
        dy = gy - y
        distance = math.hypot(dx, dy)

        # Convert goal vector to robot frame
        dx_r = math.cos(-self.estimated_yaw) * dx - math.sin(-self.estimated_yaw) * dy
        dy_r = math.sin(-self.estimated_yaw) * dx + math.cos(-self.estimated_yaw) * dy

        cmd = Twist()

        if distance > 0.2:
            cmd.linear.x = max(min_linear_speed, min(max_linear_speed, dx_r * 0.5))
            cmd.angular.z = max(-max_angular_speed, min(max_angular_speed, dy_r * 1.5))
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info("Target reached!")

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoToXY()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
