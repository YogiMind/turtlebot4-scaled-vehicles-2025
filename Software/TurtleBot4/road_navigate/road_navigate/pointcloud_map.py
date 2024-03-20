import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import cv2
import struct
import math
from nav_msgs.msg import OccupancyGrid
import time
from numba import jit


@jit(nopython=True)
def collect_points_into_grid(data_array, mapSizeWidth,mapSizeForward, grid_resolution):

        grid_size_x = int(np.ceil((mapSizeWidth / grid_resolution)))
        grid_size_y = int(np.ceil((mapSizeForward / grid_resolution)))
        map_grid = np.zeros((grid_size_y, grid_size_x, 2))  # 2 channels: accumulated intensity and point count

        xMapOffset = mapSizeWidth / 2


        for point in data_array:  
            x = point['x']
            y = point['y']
            z = point['z']
            intensity = point['intensity']

            if not (math.isnan(x) or math.isnan(y) or math.isnan(z) or math.isnan(intensity)) and \
            0 < z < mapSizeForward and -xMapOffset < x < xMapOffset:

                grid_x = int((x + xMapOffset) / grid_resolution)
                grid_y = int(z / grid_resolution)

                map_grid[grid_y, grid_x, 0] += intensity
                map_grid[grid_y, grid_x, 1] += 1

        return map_grid

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('point_cloud_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/oakd/points',
            self.point_cloud_callback,
            rclpy.qos.QoSProfile(
                reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT, 
                depth=10
            )
        )

        self.map_publisher = self.create_publisher(OccupancyGrid, '/myRoad', 1)

        self.mapSizeForward = 2.0 #map size in meters
        self.mapSizeWidth = 2.5
        self.grid_resolution = 0.01  # 1cm per pixel
        self.robot_base_frame = "oakd_link"


    def point_cloud_callback(self, msg):

        point_dtype = np.dtype([('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('intensity', 'f4')])
        data_array = np.frombuffer(msg.data, dtype=point_dtype)

        base_grid = collect_points_into_grid(data_array, self.mapSizeWidth, self.mapSizeForward, self.grid_resolution)
        
        avg_image = self.average_collected_points_in_grid(base_grid)

        filtered_image = self.filter_lines_ground(avg_image, 150)
        #cohesion_road = self.road_post_processing(filtered_image)
        #road = self.filter_lines_ground(cohesion_road, 170)

        self.map_publisher.publish(self.numpy_array_to_occupancy_grid(filtered_image))



    def numpy_array_to_occupancy_grid(self, array2d):
        
        array2d = np.rot90(array2d)

        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.frame_id = self.robot_base_frame
        occupancy_grid.info.width = array2d.shape[1]
        occupancy_grid.info.height = array2d.shape[0]
        occupancy_grid.info.resolution = self.grid_resolution
        occupancy_grid.info.origin.position.x = 0.0
        occupancy_grid.info.origin.position.y = -self.mapSizeWidth/2
        occupancy_grid.info.origin.position.z = 0.0
   
        occupancy_grid.info.origin.orientation.x = 0.0
        occupancy_grid.info.origin.orientation.y = 0.0
        occupancy_grid.info.origin.orientation.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0

        flattened_array = array2d.flatten()

        min_val = np.min(flattened_array)
        max_val = np.max(flattened_array)

        normalized_array = ((flattened_array - min_val) / (max_val - min_val)) * 100

        clamped_values = np.clip(normalized_array, 0, 100).astype(int)
        occupancy_grid.data = clamped_values.tolist()

        return occupancy_grid

    
    def average_collected_points_in_grid(self, map_grid):
        avg_image = np.zeros_like(map_grid[..., 0])
        mask = (map_grid[..., 1] != 0)
        avg_image[mask] = map_grid[..., 0][mask] / map_grid[..., 1][mask]

        return avg_image

    def filter_lines_ground(self, map, gray_scale_threshold):
        filtered_map = np.where(map < gray_scale_threshold, 0, map)
        return filtered_map

    def road_post_processing(self, map):
        # Define a kernel for dilation
        kernel = np.ones((5, 2), np.uint8)  # Adjust the kernel size based on your image and requirements

        # Apply dilation to connect white regions
        dilated_image = cv2.dilate(map, kernel, iterations=1)

        return dilated_image


def main(args=None):
    rclpy.init(args=args)
    point_cloud_subscriber = PointCloudSubscriber()
    rclpy.spin(point_cloud_subscriber)
    point_cloud_subscriber.destroy_node()
    rclpy.shutdown()


