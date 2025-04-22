#!/usr/bin/env python3
import threading
from socketserver import BaseRequestHandler, UDPServer
import struct

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header

from geometry_msgs import PoseWithCovarianceStamped
from gv_interfaces.msg import GulliViewPosition
from gv_client.gullivutil import parse_packet

GV_POSITION_TOPIC = "gv_positions"


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
            if self.listen_tag_id != "all" and det.tag_id != self.listen_tag_id:
                continue

            msg = PoseWithCovarianceStamped()

            msg.header.stamp = timestamp.to_msg()
            msg.header.frame_id = "world"

            msg.pose.pose.position.x = det.x
            msg.pose.pose.position.y = det.y
            msg.pose.pose.position.z = 0.0

            # q = quaternion_from_euler(0, 0, packet.theta)
            # msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


            # Covariance (optional — tune based on system trust)
            # 6x6 row-major covariance matrix (x, y, z, roll, pitch, yaw)
            msg.pose.covariance = [
                0.05, 0.0,  0.0,  0.0, 0.0, 0.0,  # x
                0.0, 0.05, 0.0,  0.0, 0.0, 0.0,  # y
                0.0, 0.0,  0.0,  0.0, 0.0, 0.0,  # z (no z info)
                0.0, 0.0,  0.0,  0.0, 0.0, 0.0,  # roll
                0.0, 0.0,  0.0,  0.0, 0.0, 0.0,  # pitch
                0.0, 0.0,  0.0,  0.0, 0.0, 0.1   # yaw (theta)
            ]


            print(f'[*] Received position from UDP: ({det.x}, {det.y})')

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
        publisher = self.create_publisher(GulliViewPosition, topic, 10)

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
