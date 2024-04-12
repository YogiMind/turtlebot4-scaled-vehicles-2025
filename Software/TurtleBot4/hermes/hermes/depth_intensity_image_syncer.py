import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import queue, threading
from builtin_interfaces.msg import Time
import time

def get_timestamp_seconds(msg):
    stamp = msg.header.stamp
    
    # Calculate the seconds as a float value
    seconds = stamp.sec + stamp.nanosec / 1e9

    return seconds

class SyncedImages:
    def __init__(self, depth_img, right_img):
        self.depth_img = depth_img
        self.right_img = right_img

class CaptureRoad(Node):
    def __init__(self):
        super().__init__('capture_road')
        self.bridge = CvBridge()
        self.image_subscription = self.create_subscription(
            Image,
            '/oakd/right/image_rect',
            self.rightImgCallBack,
            10)
        self.depth_subscription = self.create_subscription(
            Image,
            '/oakd/stereo/image_raw',
            self.depthImgCallback,
            10)
        
        self.intensity_publisher = self.create_publisher(Image, '/theIntensity', 10)
        self.depth_publisher = self.create_publisher(Image, '/theDepth', 10)
        

        self.depthImgQueue = queue.Queue(maxsize=100)
        self.rightImgQueue = queue.Queue(maxsize=100)
        self.syncedImagesQueue = queue.Queue(maxsize=100)
        
        msg_sync_thread = threading.Thread(target=self.sync_messages)
        msg_sync_thread.start()

        process_synced_msgs_thread = threading.Thread(target=self.process_synced_messages)
        process_synced_msgs_thread.start()


        self.get_logger().info("Image syncer has started.")


    def depthImgCallback(self, msg):
        # Add received message from '/oakd/stereo/image_raw' to queue
        try:
            self.depthImgQueue.put_nowait(msg)
        except:
            pass


    def rightImgCallBack(self, msg):
        # Add received message from '/oakd/right/image_rect' to queue
        try:
            self.rightImgQueue.put_nowait(msg)
        except:
            pass


    def sync_messages(self):
        depthImg = self.depthImgQueue.get()
        rightImg = self.rightImgQueue.get()
        # Synchronize messages based on their timestamps
        while True:
            
            
            timeDif = (get_timestamp_seconds(depthImg) - get_timestamp_seconds(rightImg))

            if abs(timeDif) < 0.07:
                syncImg=SyncedImages(depth_img=depthImg, right_img=rightImg)
                
                try:
                    self.syncedImagesQueue.put_nowait(syncImg)
                except:
                    self.get_logger().info("syncedImagesQueue is full")
                
                depthImg = self.depthImgQueue.get()
                rightImg = self.rightImgQueue.get()
            elif timeDif < 0:
                depthImg = self.depthImgQueue.get()
                self.get_logger().info("small")
            else:
                rightImg = self.rightImgQueue.get()
                self.get_logger().info("big")


    def process_synced_messages(self):
        while True:
            images = self.syncedImagesQueue.get()
            
            depth_img_msg = images.depth_img
            right_img_msg = images.right_img

            self.depth_publisher.publish(depth_img_msg)
            self.intensity_publisher.publish(right_img_msg)



def main(args=None):
    rclpy.init(args=args)
    line_detector_node = CaptureRoad()
    rclpy.spin(line_detector_node)
    line_detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
