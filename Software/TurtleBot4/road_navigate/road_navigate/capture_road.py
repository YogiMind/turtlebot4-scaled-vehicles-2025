import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from cv_bridge import CvBridgeError

from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration


class CaptureRoad(Node):
    def __init__(self):
        super().__init__('capture_road')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/oakd/right/image_rect',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Image, '/processed_image', 10)
        self.get_logger().info("Line detector node has been started.")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {e}')

        # Process the image to detect lines and separate the returned values
        processed_image, filtered_lines = self.detect_lines(cv_image)  # Note the separation here

        # Now pass only the processed image to detect curves
        processed_image = self.detect_curves(processed_image)

        # Convert the processed image back to a ROS2 message and publish it
        try:
            processed_image_msg = self.bridge.cv2_to_imgmsg(processed_image, 'bgr8')
            self.publisher.publish(processed_image_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {e}')



    def mask_upper_half(self, cv_image):
        # Define the region of interest (ROI) as the bottom half of the image
        height, width = cv_image.shape[:2]
        mask = np.zeros_like(cv_image)
        
        # Set the vertices for the polygon (mask everything above the bottom half)
        # The vertices are defined as the bottom-left, top-left, top-right, bottom-right of the desired region
        polygon = np.array([[
            (0, height),  # Bottom-left
            (0, height // 2),  # Top-left (midway point of the height)
            (width, height // 2),  # Top-right
            (width, height)  # Bottom-right
        ]], np.int32)
        
        # Fill the polygon with white color (masking the top half)
        cv2.fillPoly(mask, polygon, (255,) * cv_image.shape[2])
        
        # Apply the mask and bitwise AND to keep only the bottom half
        masked_image = cv2.bitwise_and(cv_image, mask)
        return masked_image

    def detect_lines(self, cv_image):        
        masked_image = self.mask_upper_half(cv_image)
        
        # Apply a Gaussian blur
        blurred_frame = cv2.GaussianBlur(masked_image, (7, 7), 0)
        
        # Apply Canny Edge Detection
        edges = cv2.Canny(blurred_frame, 74, 200)
        
        # Dilate the edges to make the lines appear thicker
        kernel = np.ones((1, 1), np.uint8)  # Adjust the kernel size for less dilation
        dilated_edges = cv2.dilate(edges, kernel, iterations=1)
        
        # Apply HoughLinesP to detect lines on the dilated edges
        lines = cv2.HoughLinesP(dilated_edges, 1, np.pi / 180, threshold=50, minLineLength=120, maxLineGap=100)
        
        # Filter out horizontal lines and draw them on the original image
        if lines is not None:
            filtered_lines = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # Calculate the angle of the line
                angle = np.arctan2(y2 - y1, x2 - x1) * 180.0 / np.pi
                # Only add lines that are not horizontal
                if not (abs(angle) < 10 or abs(angle) > 170):
                    filtered_lines.append(line)
                    cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 3) # Change thickness here if needed
        
        return cv_image, filtered_lines

    def detect_curves(self, cv_image):        
        masked_image = self.mask_upper_half(cv_image)
        
        # Apply a Gaussian blur
        blurred_frame = cv2.GaussianBlur(masked_image, (5, 5), 0)
        
        # Apply Canny Edge Detection
        edges = cv2.Canny(blurred_frame, 74, 220)
        
        # Find contours in the edge-detected image
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Approximate contours to polylines and draw them on the original image
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
            cv2.drawContours(cv_image, [approx], 0, (0, 255, 0), 5)
            
        return cv_image




def main(args=None):
    rclpy.init(args=args)
    line_detector_node = CaptureRoad()
    rclpy.spin(line_detector_node)
    line_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
