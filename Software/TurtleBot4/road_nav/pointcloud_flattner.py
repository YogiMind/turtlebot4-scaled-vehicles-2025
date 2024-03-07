import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
import numpy as np
import cv2
import struct
import math
from cv_bridge import CvBridge

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('point_cloud_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/oak/points',
            self.point_cloud_callback,
            rclpy.qos.QoSProfile(
                reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT, 
                depth=10
            )
        )

        self.bridge = CvBridge()
        
        
        self.image_publisher = self.create_publisher(Image, '/myRoad', 10)

        self.mapSize = 2.5 #map size in meters
        self.grid_resolution = 0.01  # 0.5cm per pixel
        #self.gray_scale_threshold = 130 # make objects dimmer than value, max is 255


    def point_cloud_callback(self, msg):
        self.get_logger().info('Received PointCloud2 message')

        for i in range(max(4, len(msg.fields))):
            self.get_logger().info(f'Field {i} name: {msg.fields[i].name}')
            self.get_logger().info(f'Field {i} datatype: {msg.fields[i].datatype}')

        base_grid = self.collect_points_into_grid(msg)
        
        avg_image = self.image_from_grid(base_grid)

        filtered_image = self.filter_lines_ground(avg_image, 120)
        cohesion_road = self.road_post_processing(filtered_image)
        road_img = self.filter_lines_ground(cohesion_road, 160)

        image_msg = self.bridge.cv2_to_imgmsg((road_img).astype(np.uint8), encoding='mono8')
        self.image_publisher.publish(image_msg)


    def collect_points_into_grid(self, msg):
        data = msg.data
        point_step = msg.point_step
        x_offset = msg.fields[0].offset
        y_offset = msg.fields[1].offset
        z_offset = msg.fields[2].offset
        intensity_offset = msg.fields[3].offset 

        grid_size_x = int(np.ceil((self.mapSize * 2 / self.grid_resolution)))
        grid_size_y = int(np.ceil((self.mapSize / self.grid_resolution)))
        image_grid = np.zeros((grid_size_y, grid_size_x, 2))  # 2 channels: accumulated intensity and point count

        for i in range(0, len(data), point_step):
            x = struct.unpack_from('f', data, i + x_offset)[0]
            y = struct.unpack_from('f', data, i + y_offset)[0]
            z = struct.unpack_from('f', data, i + z_offset)[0]
            intensity = struct.unpack_from('f', data, i + intensity_offset)[0]

            self.add_points_for_road(x, y, z, intensity, image_grid)

        return image_grid


    def add_points_for_road(self, x,y,z,intensity, image_grid):
        
        if not (math.isnan(x) or math.isnan(y) or math.isnan(z) or math.isnan(intensity)):
            if(z <= self.mapSize and z >= 0):

                grid_x = int((x + self.mapSize) / self.grid_resolution)
                grid_y = int((z) / self.grid_resolution)

                # Accumulate intensity and number of points gatheter in the corresponding grid cell
                image_grid[grid_y, grid_x, 0] += intensity
                image_grid[grid_y, grid_x, 1] += 1



    def image_from_grid(self, image_grid):
        avg_image = np.zeros_like(image_grid[..., 0])
        mask = (image_grid[..., 1] != 0)
        avg_image[mask] = image_grid[..., 0][mask] / image_grid[..., 1][mask]

        return avg_image

    def filter_lines_ground(self, img, gray_scale_threshold):
        filtered_image = np.where(img < gray_scale_threshold, 0, img)
        return filtered_image

    def road_post_processing(self, img):
        # Define a kernel for dilation
        kernel = np.ones((5, 2), np.uint8)  # Adjust the kernel size based on your image and requirements

        # Apply dilation to connect white regions
        dilated_image = cv2.dilate(img, kernel, iterations=1)

        return dilated_image


def main(args=None):
    rclpy.init(args=args)
    point_cloud_subscriber = PointCloudSubscriber()
    rclpy.spin(point_cloud_subscriber)
    point_cloud_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    print("running road_flattner")
    main()