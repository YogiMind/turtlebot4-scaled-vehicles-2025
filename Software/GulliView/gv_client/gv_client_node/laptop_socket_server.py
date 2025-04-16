#!/usr/bin/env python3
import threading
from socketserver import BaseRequestHandler, UDPServer
import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from gv_interfaces.msg import LaptopSpeed
from gv_client_node.gullivutil import parse_packet

GV_POSITION_TOPIC = "gv_laptop"

def unpack_data(buf: bytearray, start: int) -> int:
    """Helper method to unpack big-endian uint32's from a buffer."""
    return struct.unpack('>I', buf[start:start+4])[0]


class LaptopPacketHandler(BaseRequestHandler):
    """
    Request handler to unpack GulliView packets and publish data on a ROS topic.
    """
    node = None
    publisher = None
    listen_tag_id = None

    def handle(self):
        recv_buf = bytearray(self.request[0])
        print(f'[*] Received packet {recv_buf}')

        main_id = unpack_data(recv_buf, 0)
        main_speed = struct.unpack('>f', recv_buf[4:8])[0]
        main_restart_ready = struct.unpack('>?', recv_buf[8:9])[0]
        ramp_id = unpack_data(recv_buf, 9)
        ramp_speed = struct.unpack('>f', recv_buf[13:17])[0]
        ramp_restart_ready = struct.unpack('>?', recv_buf[17:18])[0]

        print(f'[*] Main ID: {main_id}, Main Speed: {main_speed}')
        print(f'[*] Ramp ID: {ramp_id}, Ramp Speed: {ramp_speed}')
        print(f'[*] Main RESTART: {main_restart_ready}, Ramp RESTART: {ramp_restart_ready}')

        safe_speed = 0.8
        if abs(main_speed) > safe_speed or abs(ramp_speed) > safe_speed:
            print('\n' + '_' * 32)
            print(f'[!] Safety Cut-off: speeds from UDP received: ({main_speed}, {ramp_speed})')
            print('_' * 32 + '\n')
            main_speed = safe_speed if main_speed > 0 else 0.01
            ramp_speed = safe_speed if ramp_speed > 0 else 0.01

        # Publish based on tag ID
        if self.listen_tag_id == main_id:
            header = Header()
            header.stamp = self.node.get_clock().now().to_msg()
            msg = LaptopSpeed(header=header, tag_id=main_id, speed=main_speed, restart=main_restart_ready)
            self.publisher.publish(msg)
        elif self.listen_tag_id == ramp_id:
            header = Header()
            header.stamp = self.node.get_clock().now().to_msg()
            msg = LaptopSpeed(header=header, tag_id=ramp_id, speed=ramp_speed, restart=ramp_restart_ready)
            self.publisher.publish(msg)


class LaptopServerNode(Node):
    def __init__(self):
        super().__init__('gulliview')
        self.get_logger().info("Started gulliview node")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 2222)
        self.declare_parameter("tag_id", "all")

        host = self.get_parameter("host").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value
        listen_tag_id = self.get_parameter("tag_id").get_parameter_value().string_value

        topic = GV_POSITION_TOPIC
        self.get_logger().info(f"Setting up publisher on {topic}")
        publisher = self.create_publisher(LaptopSpeed, topic, 10)

        # Bind handler context
        LaptopPacketHandler.node = self
        LaptopPacketHandler.publisher = publisher
        LaptopPacketHandler.listen_tag_id = listen_tag_id

        self.get_logger().info(f"Starting UDP server on {host}:{port}, listening for tag ID: {listen_tag_id}")
        self.server = UDPServer((host, port), LaptopPacketHandler)

        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.start()

    def destroy_node(self):
        self.get_logger().info("Node received shutdown signal, shutting down server")
        self.server.shutdown()
        self.get_logger().info("Server shutdown, exiting")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaptopServerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

