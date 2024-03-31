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
from abc import ABC, abstractmethod
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener


import threading, queue


class PointCloudImgPostProcessor():
    
    @staticmethod
    @jit(nopython=True)
    def collect_cloudpoints_into_grid(data_array, width, height, grid_resolution):
        grid_size_x = width
        grid_size_y = height
        grid = np.zeros((grid_size_y, grid_size_x, 2))  # 2 channels: accumulated intensity and point count
        xOffset = width*grid_resolution / 2
        zLimit = height*grid_resolution
        for point in data_array:  
            x = point['x']
            y = point['y']
            z = point['z']
            intensity = point['intensity']
            if not (math.isnan(x) or math.isnan(y) or math.isnan(z) or math.isnan(intensity)) and \
            0 < z < zLimit and -xOffset < x < xOffset:
                grid_x = int((x + xOffset) / grid_resolution)
                grid_y = int(z / grid_resolution)
                grid[grid_y, grid_x, 0] += intensity
                grid[grid_y, grid_x, 1] += 1


        return np.rot90(grid)


    @staticmethod
    def average_collected_points_in_grid(map_grid):
        avg_grid = np.zeros_like(map_grid[..., 0])
        mask = (map_grid[..., 1] != 0)
        avg_grid[mask] = map_grid[..., 0][mask] / map_grid[..., 1][mask]
        return avg_grid
    

    @staticmethod
    def filter_lines_ground(map, gray_scale_threshold):
        filtered_map = np.where(map < gray_scale_threshold, 0, map)
        return filtered_map


    def __init__(self, mapScale : float, localMapSizeForward: float, localMapSizeLeftRigt: float, fovAngle: float, visionOffset: float):
        self.mapScale = mapScale
        self.localMapSizeForward = int(np.ceil((localMapSizeForward / mapScale)))
        self.localMapSizeLeftRigt = int(np.ceil((localMapSizeLeftRigt / mapScale)))
        self.fovAngle = fovAngle
        self.visionOffset = round(visionOffset / mapScale)
        self.viewable_indices = self.get_viewable_indices()


    def getPoints(self, pointCloudData):
        base_grid = self.collect_cloudpoints_into_grid(pointCloudData, self.localMapSizeLeftRigt, self.localMapSizeForward, self.mapScale) # 2 channels: accumulated intensity and point count
        avg_grid = self.average_collected_points_in_grid(base_grid)  # 1 channel: average intensity
        filtered_grid = self.filter_lines_ground(avg_grid, 150)
        
        #post-processing can be added on the grid/image

        points = self.recreatePoints(filtered_grid)
        return points
    

    def recreatePoints(self, data):
        points = np.zeros((self.viewable_indices.shape[0], 4), dtype=float)
        points[:, [1, 0]] = self.viewable_indices 
        points[:, 3] = data[self.viewable_indices[:, 0], self.viewable_indices[:, 1]]
        points[:, :2] *= self.mapScale
        points[:, :2] += self.mapScale / 2.0
        points[:, 1] -= self.localMapSizeLeftRigt * self.mapScale / 2.0
        return points


    def get_viewable_indices(self):
        rows = self.localMapSizeLeftRigt
        cols = self.localMapSizeForward

        y, x = np.ogrid[:rows, :cols]
        xangle = np.degrees(np.arctan2(y - rows // 2, x))
        mask = (xangle >= -self.fovAngle) & (xangle <= self.fovAngle) & (x >= self.visionOffset)

        Y, X = np.meshgrid(np.arange(cols), np.arange(rows))
        indeces = np.stack((X, Y), axis=-1)
        
        return indeces[mask]


class Map():
    def __init__(self, mapData: np.array, origin: np.array, resolution: float):
        self.mapData = mapData
        self.origin = origin
        self.resolution = resolution


class Subscriber(ABC):
    @abstractmethod
    def update(context):
        pass

class MapHandler():
    
    map = np.zeros((1, 1), dtype=np.int32)
    origin = np.array([0, 0]) 
    underlyingMapGrid = np.zeros((1, 1, 2), dtype=np.int32)
    lock = threading.RLock()
    subscribers = [Subscriber]


    def __init__(self, mapScale, logger):
        self.mapScale = mapScale
        self.logger = logger


    def __notifySubscribers(self):
        for sub in self.subscribers:
            sub.update(self)


    def __formMap(self):
        avg_map = np.zeros_like(self.underlyingMapGrid[..., 0])
        
        mask = (self.underlyingMapGrid[..., 1] == 0)
        avg_map[mask] = -1
        
        mask = (self.underlyingMapGrid[..., 1] != 0)
        avg_map[mask] = self.underlyingMapGrid[..., 0][mask] / self.underlyingMapGrid[..., 1][mask]
        
        low = 22
        mask = (avg_map >= low) & (avg_map <= 250)
        avg_map[mask] = 255
        
        mask = (avg_map >= 0) & (avg_map < low)
        avg_map[mask] = 0

        self.map = avg_map


    def __updateMapSize(self, x, y):
        map_x = np.round(x + self.origin[0]).astype(int)
        map_y = np.round(y + self.origin[1]).astype(int)

        min_map_x = np.min(map_x)
        max_map_x = np.max(map_x)
        min_map_y = np.min(map_y)
        max_map_y = np.max(map_y)

        hight = self.underlyingMapGrid.shape[0]
        width = self.underlyingMapGrid.shape[1]

        # Adjust map size and origin if needed for x-axis
        if min_map_x < 0:
            col_add_size = 2 * np.abs(min_map_x)
            zeros_columns = np.zeros((self.underlyingMapGrid.shape[0], col_add_size, 2))
            self.underlyingMapGrid = np.hstack((zeros_columns, self.underlyingMapGrid))
            self.origin[0] += col_add_size
        if max_map_x >= width:
            col_add_size = 2 * np.abs(max_map_x - (width - 1))
            zeros_columns = np.zeros((self.underlyingMapGrid.shape[0], col_add_size, 2))
            self.underlyingMapGrid = np.hstack((self.underlyingMapGrid, zeros_columns))

        # Adjust map size and origin if needed for y-axis
        if min_map_y < 0:
            row_add_size = 2 * np.abs(min_map_y)
            zeros_rows = np.zeros((row_add_size, self.underlyingMapGrid.shape[1], 2))
            self.underlyingMapGrid = np.vstack((zeros_rows, self.underlyingMapGrid))
            self.origin[1] += row_add_size
        if max_map_y >= hight:
            row_add_size = 2 * np.abs(max_map_y - (hight - 1))
            zeros_rows = np.zeros((row_add_size, self.underlyingMapGrid.shape[1], 2))
            self.underlyingMapGrid = np.vstack((self.underlyingMapGrid, zeros_rows))


    def addSubscriber(self, subscriber: Subscriber):
        with self.lock:
            self.subscribers.append(subscriber)


    def getGlobalMap(self) -> Map:
        with self.lock:
            map = self.map.copy()
            return Map(map, self.origin, self.mapScale)
    

    def addPoints(self, points: np.array):
        with self.lock:
            points = np.array(points)

            x = np.round(points[:, 0] / self.mapScale).astype(int)
            y = np.round(points[:, 1] / self.mapScale).astype(int)
            values = points[:, 3]
            
            self.__updateMapSize(x, y)
            #Convert coordinates to map coords
            map_x = np.round(x + self.origin[0]).astype(int)
            map_y = np.round(y + self.origin[1]).astype(int)

            self.underlyingMapGrid[map_y, map_x, 0] += values
            self.underlyingMapGrid[map_y, map_x, 1] += 1

            self.__formMap()
            self.__notifySubscribers()


class MapServer(Subscriber):

    @staticmethod
    def map_to_occupancy_grid(map: Map, frame):

        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.frame_id = frame
        occupancy_grid.info.width = map.mapData.shape[1]
        occupancy_grid.info.height = map.mapData.shape[0]
        occupancy_grid.info.resolution = map.resolution
        occupancy_grid.info.origin.position.x = -map.origin[0] * map.resolution
        occupancy_grid.info.origin.position.y = -map.origin[1] * map.resolution
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.x = 0.0
        occupancy_grid.info.origin.orientation.y = 0.0
        occupancy_grid.info.origin.orientation.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0

        flattened_array = map.mapData.flatten()

        min_val = np.min(flattened_array)
        max_val = np.max(flattened_array)

        normalized_array = ((flattened_array - min_val) / (max_val - min_val)) * 100
        clamped_values = np.clip(normalized_array, 0, 100).astype(int)
        occupancy_grid.data = clamped_values.tolist()
        
        return occupancy_grid

    def __init__(self, rosNode, timePeriod, publishedTopicName, frame):
        self.publisher = rosNode.create_publisher(OccupancyGrid, publishedTopicName, 10)
        self.timer = rosNode.create_timer(timePeriod, self.__publish)
        self.frame = frame
        self.lock = threading.Lock()
        self.map = None

    def __publish(self):
        mapData = None
        with self.lock:
            if(self.map):
                mapData = self.map
        if mapData:
            self.publisher.publish(mapData)

    def update(self, context : MapHandler):
        mapData = self.map_to_occupancy_grid(context.getGlobalMap(), self.frame)
        with self.lock:
            self.map = mapData
            
class RoadMap(Node):
    """
    Represents a road map for the TurtleBot4 navigation system.

    This class provides functionality for transforming points, processing point cloud data,
    and adding points to the map.

    Attributes:
        publishedTopic (str): The topic on which the road map is published.
        pointCloudTopic (str): The topic on which the point cloud data is received.
        robot_base_frame (str): The frame of the robot's base.
        robot_map_frame (str): The frame of the map.
        localMapSizeForward (float): The size of the local map in the forward direction.
        localMapSizeLeftRigt (float): The size of the local map in the left-right direction.
        grid_resolution (float): The resolution of the grid used for mapping.
        fov (float): The field of view of the camera.
        visionOffset (float): The offset of the camera from the robot's base.
        mapServerTimePeriod (float): The time period for updating the map server.
        tf_buffer (Buffer): The buffer for storing transforms.
        tf_listener (TransformListener): The listener for transforms.
        pointCloudPostProcessor (PointCloudImgPostProcessor): The post-processor for point cloud data.
        mapHandler (MapHandler): The handler for the map.
        mapServer (MapServer): The map server.
        queue (Queue): The queue for storing point cloud messages.
        subscriptionPointCloud (Subscription): The subscription for point cloud messages.
    """

    @staticmethod
    def transform_points(points, transform):
        """
        Transforms the given points using the specified transform.

        Args:
            points (numpy.ndarray): The points to be transformed.
            transform (geometry_msgs.msg.TransformStamped): The transform to apply.

        Returns:
            numpy.ndarray: The transformed points.
        """
        translation = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
        rotation_quaternion = np.array([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
        
        # Convert quaternion to rotation matrix
        x, y, z, w = rotation_quaternion
        R = np.array([[1 - 2*(y**2 + z**2), 2*(x*y - z*w), 2*(x*z + y*w)],
                    [2*(x*y + z*w), 1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
                    [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x**2 + y**2)]])

        points_rotated = np.dot(points, R.T)

        points_transformed = points_rotated + translation
        return points_transformed
    
    
    def __init__(self):
        """
        Initializes a new instance of the RoadMap class.
        """
        super().__init__('RoadMap')
        
        self.publishedTopic = '/myRoad'
        self.pointCloudTopic = '/oakd/points'
        self.robot_base_frame = "oakd_link"
        self.robot_map_frame = "map"
        self.localMapSizeForward = 1.9 #m
        self.localMapSizeLeftRigt = 2.3 #m
        self.grid_resolution = 0.02  #m
        self.fov = 53.0 #degrees
        self.visionOffset = 0.65 #m
        self.mapServerTimePeriod = 1.0

        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=60))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pointCloudPostProcessor = PointCloudImgPostProcessor(self.grid_resolution, self.localMapSizeForward, self.localMapSizeLeftRigt, self.fov, self.visionOffset)
        self.mapHandler = MapHandler(self.grid_resolution, self.get_logger())
        self.mapServer = MapServer(self, self.mapServerTimePeriod, self.publishedTopic, self.robot_map_frame)
        self.mapHandler.addSubscriber(self.mapServer)
        
        

        self.queue = queue.Queue(maxsize=5) 
        self.start_processing_pointcloud_queue()
	
        self.hold_for_transform()

        #start taking pointcloud messages
        self.subscriptionPointCloud = self.create_subscription(
            PointCloud2,
            self.pointCloudTopic,
            self.point_cloud_callback,
            rclpy.qos.QoSProfile(
                reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT, 
                depth=10
            )
        )


    def hold_for_transform(self):
        """
        Waits for the transform from the robot's base frame to the map frame to become available.
        """
        while not self.tf_buffer.can_transform(target_frame=self.robot_map_frame, source_frame=self.robot_base_frame, time=rclpy.time.Time()):
            self.get_logger().info("Transform " + str(self.robot_base_frame) + " to " + str(self.robot_map_frame) + " not available, retrying...")
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info("Transform " + str(self.robot_base_frame) + " to " + str(self.robot_map_frame) + " found")


    
    def point_cloud_callback(self, msg):
        """
        Callback function for processing point cloud messages.

        Args:
            msg (sensor_msgs.msg.PointCloud2): The point cloud message.
        """
        try:
            self.queue.put_nowait(msg)  # Enqueue the message
        except:
            pass

    def process_pointcloud_queue(self):
        """
        Processes the point cloud messages in the queue.
        """
        while True:
            msg = self.queue.get()  # Dequeue the message
            self.add_to_map(msg)

    def start_processing_pointcloud_queue(self):
        """
        Starts the thread for processing the point cloud queue.
        """
        processing_thread = threading.Thread(target=self.process_pointcloud_queue)
        processing_thread.start()

    def add_to_map(self, msg):
        """
        Adds the points from the point cloud message to the map.

        Args:
            msg (sensor_msgs.msg.PointCloud2): The point cloud message.
        """
        self.get_logger().info("in add to map")

        point_dtype = np.dtype([('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('intensity', 'f4')])
        pointcloud_data_array = np.frombuffer(msg.data, dtype=point_dtype)
        pointcloud_timestamp = msg.header.stamp

        points = self.pointCloudPostProcessor.getPoints(pointcloud_data_array)

        try:
            transform = self.tf_buffer.lookup_transform(target_frame=self.robot_map_frame, source_frame=self.robot_base_frame, time=pointcloud_timestamp, timeout=rclpy.duration.Duration(seconds=5))
        except:
            self.get_logger().info("transfrom failed, trying again")
            return

        s=time.time()

        #x,y,z cols only
        points[:, :3] = self.transform_points(points[:, :3], transform)
        self.mapHandler.addPoints(points)

        e=time.time()
        self.get_logger().info(str(e-s))
        




def main(args=None):
    rclpy.init(args=args)
    roadMap = RoadMap()
    try:
        rclpy.spin(roadMap)
    except KeyboardInterrupt:
        pass
    finally:
        roadMap.destroy_node()
        rclpy.shutdown()

