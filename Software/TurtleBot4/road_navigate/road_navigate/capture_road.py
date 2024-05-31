import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import threading
import collections

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
        self.depth_publisher = self.create_publisher(Image, '/theLinesDepth', 10)
        self.preview_lines_publisher = self.create_publisher(Image, '/linesPreview', 10)


        self.depthImgQueue = collections.deque(maxlen=100)
        self.rightImgQueue = collections.deque(maxlen=100)
        self.syncedImagesQueue = collections.deque(maxlen=100)
        
        msg_sync_thread = threading.Thread(target=self.sync_messages)
        msg_sync_thread.start()

        process_synced_msgs_thread = threading.Thread(target=self.process_synced_messages)
        process_synced_msgs_thread.start()


        self.get_logger().info("Line detector node has been started.")


    def depthImgCallback(self, msg):
        # Add received message from '/oakd/stereo/image_raw' to queue
        try:
            self.depthImgQueue.append(msg)  # Append to the right side
        except IndexError:
            # Handle full deque if maxlen is reached
            pass  #


    def rightImgCallBack(self, msg):
        # Add received message from '/oakd/right/image_rect' to queue
        try:
            self.rightImgQueue.append(msg)
        except IndexError:
            pass  # Optionally handle the scenario where deque is full, if needed



    def sync_messages(self):
        depthImg = None
        rightImg = None

        while True:
            if not depthImg and self.depthImgQueue:
                depthImg = self.depthImgQueue.popleft()
            if not rightImg and self.rightImgQueue:
                rightImg = self.rightImgQueue.popleft()

            if depthImg and rightImg:
                timeDif = get_timestamp_seconds(depthImg) - get_timestamp_seconds(rightImg)

                if abs(timeDif) < 0.07:  # Synchronization threshold
                    syncImg = SyncedImages(depth_img=depthImg, right_img=rightImg)
                    self.syncedImagesQueue.append(syncImg)
                    depthImg = None  # Reset for next synchronization
                    rightImg = None
                elif timeDif < 0:
                    depthImg = None  # depth image is older, fetch next
                else:
                    rightImg = None  # right image is older, fetch next
            else:
                break  # Exit loop if one or both queues are empty
    def process_synced_messages(self):
        while True:
            if not self.syncedImagesQueue:
                break  # Exit if the queue is empty
            
            images = self.syncedImagesQueue.popleft()
            cv_image = None
            depth_image = None
            try:
                cv_image = self.bridge.imgmsg_to_cv2(images.right_img, 'bgr8')
                depth_image = self.bridge.imgmsg_to_cv2(images.depth_img, desired_encoding='passthrough')
                
       
                final_image, line_coordinates, curve_contours = self.apply_preprocessing_steps(cv_image)

                if line_coordinates or curve_contours:
                    processed_image_msg = self.bridge.cv2_to_imgmsg(final_image, 'bgr8')
                    processed_image_msg.header = images.right_img.header
                    self.preview_lines_publisher.publish(processed_image_msg)

                    distance_threshold = 3000
                    depth_combined_image = np.zeros_like(depth_image)
                    if line_coordinates:
                        depth_lines_image = self.get_depth_image_for_lines(depth_image, line_coordinates, distance_threshold)
                        depth_combined_image = np.maximum(depth_combined_image, depth_lines_image)
                    if curve_contours:
                        depth_curves_image = self.get_depth_image_for_curves(depth_image, curve_contours, distance_threshold)
                        depth_combined_image = np.maximum(depth_combined_image, depth_curves_image)

                    depth_image_msg = self.bridge.cv2_to_imgmsg(depth_combined_image, encoding="passthrough")
                    depth_image_msg.header = images.depth_img.header
                    self.depth_publisher.publish(depth_image_msg)
                    self.intensity_publisher.publish(images.right_img)
            except CvBridgeError as e:
                self.get_logger().error(f'Failed to convert images: {e}')
                continue  # Skip to the next iteration if there's an error


    def apply_preprocessing_steps(self, cv_image):
        # Apply mask to upper half just once
        masked_image = self.mask_upper_half(cv_image)
        
        # Assuming you want curves drawn on the image that already has lines:
        final_image, line_coordinates, curve_contours = self.detect_lines_and_curves(masked_image)
        
        # Now final_image contains both lines and curves
        return final_image, line_coordinates, curve_contours

    def mask_upper_half(self, cv_image):
        height, width = cv_image.shape[:2]
        mask = np.zeros_like(cv_image)
        polygon = np.array([[
            (0, height),
            (0, height // 2),
            (width, height // 2),
            (width, height)
        ]], np.int32)
        cv2.fillPoly(mask, polygon, (255,) * cv_image.shape[2])
        masked_image = cv2.bitwise_and(cv_image, mask)
        return masked_image

    def detect_lines_and_curves(self, masked_image, original_image=None):
        blurred_frame = cv2.GaussianBlur(masked_image, (7, 7), 0)
        edges = cv2.Canny(blurred_frame, 74, 200)
        kernel = np.ones((1, 1), np.uint8)
        dilated_edges = cv2.dilate(edges, kernel, iterations=1)

        drawing_base = original_image if original_image is not None else masked_image

        lines = cv2.HoughLinesP(dilated_edges, 1, np.pi / 180, threshold=150, minLineLength=50, maxLineGap=300)

        filtered_lines = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                angle = np.arctan2(y2 - y1, x2 - x1) * 180.0 / np.pi
                if not (abs(angle) < 10 or abs(angle) > 170):
                    filtered_lines.append(line)
                    cv2.line(drawing_base, (x1, y1), (x2, y2), (0, 255, 0), 3)

        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        approx_contours = []

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
            approx_contours.append(approx)  # Add to the list
            cv2.drawContours(drawing_base, [approx], 0, (0, 255, 0), 5)

        return drawing_base, filtered_lines, approx_contours
    


    def get_depth_image_for_lines(self, depth_image, line_coordinates, distance_threshold=1.0):
        try:            
            line_depth_image = np.zeros_like(depth_image)
            for line in line_coordinates:
                x1, y1, x2, y2 = line[0]
                cv2.line(line_depth_image, (x1, y1), (x2, y2), color=255, thickness=5)
            
            # Use the line image as a mask to copy values from the original depth image
            line_depth_image = np.where(line_depth_image == 255, depth_image, 0)
            
            # Filter out depths beyond the threshold
            line_depth_image = np.where(line_depth_image <= distance_threshold, line_depth_image, 0)
            
            return line_depth_image
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert depth message to CV image: {e}')
            return None

    def get_depth_image_for_curves(self, depth_image, curve_contours, distance_threshold=1.0):
        try:
            curve_depth_image = np.zeros_like(depth_image)
            for cnt in curve_contours:
                cv2.drawContours(curve_depth_image, [cnt], -1, color=255, thickness=-1)
            
            curve_depth_image = np.where(curve_depth_image == 255, depth_image, 0)
            
            # Filter out depths beyond the threshold
            curve_depth_image = np.where(curve_depth_image <= distance_threshold, curve_depth_image, 0)
            
            return curve_depth_image
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert depth message to CV image: {e}')
            return None



def main(args=None):
    rclpy.init(args=args)
    line_detector_node = CaptureRoad()
    rclpy.spin(line_detector_node)
    line_detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()