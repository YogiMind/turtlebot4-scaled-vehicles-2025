#!/usr/bin/env python3
import threading
from socketserver import BaseRequestHandler, UDPServer
import struct

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
import math
import numpy as np


from gv_client.gullivutil import parse_packet

GV_POSITION_TOPIC = "/raphael/gv_pose"

CAMERA_CENTERS = {
    0: (2643, 1664),
    1: (2679, 3658),
    2: (2619, 5887),
    3: (2681, 7937)
}

def unpack_data(buf: bytearray, start: int) -> int:
    """Helper method to unpack big-endian uint32's from a buffer."""
    return struct.unpack('>I', buf[start:start+4])[0]


class GulliViewPacketHandler(BaseRequestHandler):
    """
    Request handler to unpack GulliView packets and publish data on a ROS2 topic.
    """
    node = None
    publisher = None
    listen_tag_id = None

    def handle(self):
        recv_buf = bytearray(self.request[0])
        packet = parse_packet(recv_buf)

        timestamp = Time(seconds=packet.header.timestamp / 1000)

        for det in packet.detections:
            if self.listen_tag_id != "all" and det.tag_id != int(self.listen_tag_id):
                continue

            msg = PoseWithCovarianceStamped()

            msg.header.stamp = timestamp.to_msg()
            msg.header.frame_id = "map"

            msg.pose.pose.position.x = det.x / 1298.0
            msg.pose.pose.position.y = det.y / 1298.0
            msg.pose.pose.position.z = 0.0

            q = Quaternion()
            q.z = math.sin(det.theta / 2.0)
            q.w = math.cos(det.theta / 2.0)
            msg.pose.pose.orientation = q



            # Set covariance according to position?
            cam_x, cam_y = CAMERA_CENTERS[det.camera_id]
            dx = det.x - cam_x
            dy = det.y - cam_y
            distance = np.sqrt(dx**2 + dy**2)

            base_variance = 0.01  # small for close distances
            scale_factor = 1e-6   # tune this experimentally

            cov = base_variance + scale_factor * distance

            # Covariance (optional — tune based on system trust)
            # 6x6 row-major covariance matrix (x, y, z, roll, pitch, yaw)
            cov_matrix = [0.0]*36
            cov_matrix[0] = cov  # x variance
            cov_matrix[7] = cov  # y variance
            cov_matrix[35] = 0.1  # y variance

            msg.pose.covariance = cov_matrix

            print(f'[*] Received position from UDP: Tag={det.tag_id}, Pose=({det.x}, {det.y})')

            self.publisher.publish(msg)


class GulliViewServerNode(Node):
    def __init__(self):
        super().__init__('gulliview')
        self.get_logger().info("Started gulliview node")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 2121)
        self.declare_parameter("tag_id", "all")

        host = self.get_parameter("host").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value
        listen_tag_id = self.get_parameter("tag_id").get_parameter_value().string_value

        topic = GV_POSITION_TOPIC
        self.get_logger().info(f"Setting up publisher on {topic}")
        publisher = self.create_publisher(PoseWithCovarianceStamped, topic, 10)

        # Bind handler context
        GulliViewPacketHandler.node = self
        GulliViewPacketHandler.publisher = publisher
        GulliViewPacketHandler.listen_tag_id = listen_tag_id

        self.get_logger().info(f"Starting UDP server on {host}:{port}, listening for tag ID: {listen_tag_id}")
        self.server = UDPServer((host, port), GulliViewPacketHandler)

        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.start()

    def destroy_node(self):
        self.get_logger().info("Node received shutdown signal, shutting down server")
        self.server.shutdown()
        self.get_logger().info("Server shutdown, exiting")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GulliViewServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
