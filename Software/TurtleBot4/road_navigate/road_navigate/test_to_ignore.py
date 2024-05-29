from numpy.core.multiarray import array as array
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import cv2
import math
from nav_msgs.msg import OccupancyGrid
import time
from numba import jit
from abc import ABC, abstractmethod
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import threading, queue
import json



globalLogger = None


def transform_points(points, transform):
    
    translation = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
    rotation_quaternion = np.array([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])

    # Convert quaternion to rotation matrix
    x, y, z, w = rotation_quaternion
    R = np.array([[1 - 2*(y**2 + z**2), 2*(x*y - z*w), 2*(x*z + y*w)],
                [2*(x*y + z*w), 1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
                [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x**2 + y**2)]])
    # Perform rotation
    points_rotated = np.dot(points, R.T)
    # Perform translation
    points_transformed = points_rotated + translation
    return points_transformed


class PointCloudPostProcessor(ABC):
    @abstractmethod
    def getPoints(self, pointcloud_data, pointcloud_timestamp, tf_buffer) -> np.array:
        pass



class MapHandlerSubscriber(ABC):
    @abstractmethod
    def updateGlobalMap(context):
        pass


class DerivedMapMakerSubscriber(ABC):
    @abstractmethod
    def updateDerivedMaps(context):
        pass


class MapPostProcessor(ABC):
    @abstractmethod
    def processMap(self ,map : np.array) -> np.array:
        pass



class Map():
    def __init__(self, mapData: np.array, origin: np.array, resolution: float):
        self.mapData = mapData
        self.origin = origin
        self.resolution = resolution



class MapHandler(ABC):
    @abstractmethod
    def getMap(self) -> Map:
        pass

    @abstractmethod
    def addSubscriber(self, subscriber: MapHandlerSubscriber):
        pass

    @abstractmethod
    def addPoints(self, points: np.array):
        pass


class TransfromPointCloudPostProcessor(PointCloudPostProcessor):
    
    def __init__(self, robot_map_frame, robot_base_frame, visionRangeForward: float, visionRangeSide: float) -> None:
        self.robot_map_frame = robot_map_frame
        self.robot_base_frame = robot_base_frame
        self.visionRange = visionRangeForward
        self.visionRangeSide = visionRangeSide
    
    def __extractPointCloud(self, points):
        x = points['x'].reshape(-1, 1)
        y = points['y'].reshape(-1, 1)
        z = points['z'].reshape(-1, 1)
        intensity = points['intensity'].reshape(-1, 1)

        points = np.hstack((x, y, z, intensity))
        nan_indices = np.isnan(points).any(axis=1)
        points = points[~nan_indices]
        return points

    def getPoints(self, pointCloud, pointcloud_timestamp, pointcloud_frame, tf_buffer):
        baseToMapTransform= tf_buffer.lookup_transform(target_frame=self.robot_map_frame, source_frame=self.robot_base_frame, time=pointcloud_timestamp, timeout=rclpy.duration.Duration(seconds=5))
        camToBaseTransfrom = tf_buffer.lookup_transform(target_frame=self.robot_base_frame, source_frame=pointcloud_frame, time=pointcloud_timestamp, timeout=rclpy.duration.Duration(seconds=5))

        cloudPoints = self.__extractPointCloud(pointCloud)
        #filter = cloudPoints[:, 3] > 175
        #cloudPoints = cloudPoints[filter]

        pointsXYZ = cloudPoints[:, [0,1,2]]

        basePointsXYZ = transform_points(pointsXYZ, camToBaseTransfrom)
        basePointsXYZI = np.hstack((basePointsXYZ, cloudPoints[:, 3].reshape(-1, 1)))



        filter = basePointsXYZI[:,0] < self.visionRange
        basePointsXYZI = basePointsXYZI[filter]

        filter = (basePointsXYZI[:,1] < self.visionRangeSide) & (basePointsXYZI[:,1] > -self.visionRangeSide)
        basePointsXYZI = basePointsXYZI[filter]

        basePointsXYZ = basePointsXYZI[:,[0,1,2]]
        mapPointsXYZ = transform_points(basePointsXYZ, baseToMapTransform)

        mapPointsXYZI = np.hstack((mapPointsXYZ, basePointsXYZI[:,3].reshape(-1, 1)))
       
        return mapPointsXYZI
    

class PointCountTrackMapHandler(MapHandler):
    
    underlyingMapGrid = np.zeros((1, 1), dtype=np.int32)
    origin = np.array([0, 0], dtype=int)  # Origin point
    map = np.zeros((1, 1), dtype=np.int8)
    localMap = np.zeros((1, 1), dtype=np.int32)
    lock = threading.RLock()
    subscribers = [MapHandlerSubscriber]
    


    def __init__(self, mapScale, initWitdth, initHeight, roadIntensity, mapPostProcessor: MapPostProcessor):
        self.mapScale = mapScale
        self.mapPostProcessor = mapPostProcessor
        self.roadIntensity = roadIntensity

        initHeight = initHeight / 4
        initWitdth = initWitdth / 4

        xy = np.round(np.array([[initHeight, initWitdth], [ -initHeight,-initWitdth]])[:,[0,1]]/self.mapScale).astype(int)

        self.__updateMapSize(xy)


    def __notifySubscribers(self):
        for sub in self.subscribers:
            sub.updateGlobalMap(self)


    def __postProcessMap(self):
        self.map = self.mapPostProcessor.processMap(self.map)

    def __formMap(self):
        
        map = self.underlyingMapGrid.copy()
        filter = map > 35
        map[filter] = 100
        self.map = map

            

    def __updateMapSize(self, xy):
        x = xy[:,0].astype(int)
        y = xy[:,1].astype(int)
        

        map_x = x + self.origin[0]
        map_y = y + self.origin[1]

        min_map_x = np.min(map_x)
        max_map_x = np.max(map_x)
        min_map_y = np.min(map_y)
        max_map_y = np.max(map_y)

        hight = self.underlyingMapGrid.shape[0]
        width = self.underlyingMapGrid.shape[1]

        # Adjust map size and origin if needed for x-axis
        if min_map_x < 0:
            col_add_size = 2 * np.abs(min_map_x)
            newCols = np.full((self.underlyingMapGrid.shape[0], col_add_size), -1)
            self.underlyingMapGrid = np.hstack((newCols, self.underlyingMapGrid))
            self.origin[0] += col_add_size
        if max_map_x >= width:
            col_add_size = 2 * np.abs(max_map_x - (width - 1))
            newCols = np.full((self.underlyingMapGrid.shape[0], col_add_size), -1)
            self.underlyingMapGrid = np.hstack((self.underlyingMapGrid, newCols))

        # Adjust map size and origin if needed for y-axis
        if min_map_y < 0:
            row_add_size = 2 * np.abs(min_map_y)
            newRows = np.full((row_add_size, self.underlyingMapGrid.shape[1]), -1)
            self.underlyingMapGrid = np.vstack((newRows, self.underlyingMapGrid))
            self.origin[1] += row_add_size
        if max_map_y >= hight:
            row_add_size = 2 * np.abs(max_map_y - (hight - 1))
            newRows = np.full((row_add_size, self.underlyingMapGrid.shape[1]), -1)
            self.underlyingMapGrid = np.vstack((self.underlyingMapGrid, newRows))


    def addSubscriber(self, subscriber: MapHandlerSubscriber):
        with self.lock:
            self.subscribers.append(subscriber)


    def getMap(self) -> Map:
        with self.lock:
            return Map(self.map, self.origin, self.mapScale)
    

    def addPoints(self, points: np.array):
        with self.lock:
            #assumes point layout [Mx,My,Mz,Bx,By,Bz, Intensity] where "M" stands for Map frame and "B" for Base frame
            mappingPoints = np.array(points)
            #separate points meant for mapping, and points meant for extra range of the robot's view distance. 


            #Scale points to map
            mappingPoints[:,[0,1]] = np.round(mappingPoints[:, [0,1]] / self.mapScale).astype(int)
           
            #Update map size if points outside current map
            xy = mappingPoints[:,[0,1]]
            self.__updateMapSize(xy)

            #Convert coordinates to map coords
            mappingPoints[:, [0,1]] += self.origin

            filter = mappingPoints[:, 3] > self.roadIntensity
                        
            road = mappingPoints[filter]
            ground = mappingPoints[~filter]

            
            cellCounter = np.zeros_like(self.underlyingMapGrid)
            np.add.at(cellCounter, (road[:, 1].astype(int), road[:, 0].astype(int)), 3)
            np.add.at(cellCounter, (ground[:, 1].astype(int), ground[:, 0].astype(int)), -1)

            roadFilter = cellCounter > 0
            groundFilter = cellCounter < 0

            self.underlyingMapGrid[roadFilter] += 8
            self.underlyingMapGrid[groundFilter] -= 4

            mask = self.underlyingMapGrid < 0
            self.underlyingMapGrid[mask & groundFilter] = 0 #keep unknown cells at -1

            mask = self.underlyingMapGrid > 100
            self.underlyingMapGrid[mask] = 100

            self.__formMap()
            self.__postProcessMap()
            self.__notifySubscribers()



class DerivedMapMaker(MapHandlerSubscriber):

    subscribers = [DerivedMapMakerSubscriber]

    def __init__(self, tfBuffer, robot_map_frame, robot_base_frame, mapSizeMeters):
        self.lock = threading.RLock()
        self.map = None
        self.mapTemp = None
        self.robot_map_frame = robot_map_frame
        self.robot_base_frame = robot_base_frame
        self.tfBuffer = tfBuffer

        self.mapSizeMeters = mapSizeMeters

        processing_thread = threading.Thread(target=self.__retransformLocalMap)
        processing_thread.start()


    def __retransformLocalMap(self):
        global globalLogger
        while True:
            with self.lock:
                if(self.mapTemp):
                    self.map = Map(self.mapTemp.mapData.copy(), self.mapTemp.origin.copy(), self.mapTemp.resolution)
                    self.localMapSize = self.localMapSizeTemp
                else:
                    continue
            
            try:
                baseToMapTransform = self.tfBuffer.lookup_transform(target_frame=self.robot_map_frame, source_frame=self.robot_base_frame, time=rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5))
                self.__formLocalMap(baseToMapTransform)
                self.__formPositionCenteredGlobalMap(baseToMapTransform)
                self.__notifySubscribers()
            except Exception as e:
                globalLogger.info("error: " + str(e))


    def __notifySubscribers(self):
        for sub in self.subscribers:
            sub.updateDerivedMaps(self)

    @staticmethod
    def __select_square_around_center(arr, center, radius):

        size = radius * 2 + 1


        x, y = center
        half_size = size // 2
        
        # Calculate the indices of the square
        start_x = max(0, x - half_size)
        end_x = min(arr.shape[0], x + half_size + 1)
        start_y = max(0, y - half_size)
        end_y = min(arr.shape[1], y + half_size + 1)
        
        # Create the square
        square = np.full((size, size), -1)
        
        # Calculate the overlap of the square with the array
        arr_start_x = max(0, half_size - x)
        arr_end_x = min(size, half_size + (arr.shape[0] - x))
        arr_start_y = max(0, half_size - y)
        arr_end_y = min(size, half_size + (arr.shape[1] - y))
        
        # Fill the overlapped portion with values from the array
        square[arr_start_y:arr_end_y, arr_start_x:arr_end_x] = arr[start_y:end_y, start_x:end_x]
        
        return square

    def __formPositionCenteredGlobalMap(self, baseToMapTransform):
        translation = np.array([baseToMapTransform.transform.translation.x, baseToMapTransform.transform.translation.y])
        
        mapXYCenter = (translation / self.map.resolution).astype(int) + self.map.origin
        radius = int(1.6 / self.map.resolution)
        
        map = self.__select_square_around_center(self.map.mapData, mapXYCenter, radius)

        self.positionCenteredGlobalMap = Map(map.copy(), translation + radius, self.map.resolution)

    def __formLocalMap(self, baseToMapTransform):
        

        local_map_size = self.localMapSize

        # Initialize local map filled with -1
        local_map_data = np.full((local_map_size, local_map_size), -1, dtype=np.int8)

        rows, cols = local_map_data.shape
        local_map_indices = np.indices((rows, cols)).reshape(2, -1).T


        # Calculate the center index of the local map
        center_local = local_map_size // 2

        xym = (local_map_indices - center_local)
        xym = xym * self.map.resolution + self.map.resolution
        localMapPoints = np.hstack((xym, np.zeros((xym.shape[0],1))))

        MapPoints = transform_points(localMapPoints, baseToMapTransform)[:,[0,1]]


        map_indices_i = ((MapPoints[:, 1] / self.map.resolution) + self.map.origin[1]).astype(int)
        map_indices_j = ((MapPoints[:, 0] / self.map.resolution) + self.map.origin[0]).astype(int)


        # Mask out of bounds indices
        valid_indices_mask = (map_indices_i >= 0) & (map_indices_i < self.map.mapData.shape[0]) & (map_indices_j >= 0) & (map_indices_j < self.map.mapData.shape[1])


        # Set valid map values to the local map
        local_map_data[local_map_indices[:,1][valid_indices_mask], local_map_indices[:,0][valid_indices_mask]] = self.map.mapData[map_indices_i[valid_indices_mask], map_indices_j[valid_indices_mask]]

        self.localMap = Map(local_map_data.copy(), np.array([local_map_data.shape[1] //2, local_map_data.shape[0] //2]), self.map.resolution)


    
    def updateGlobalMap(self, context : PointCountTrackMapHandler):
        mapData = context.getMap()
        #localMapData = self.map_to_occupancy_grid(context.getLocalMap(), "base_link")
        with self.lock:
            self.mapTemp = mapData
            self.localMapSizeTemp = np.ceil(self.mapSizeMeters / mapData.resolution).astype(int)

    def addSubscriber(self, subscriber: MapHandlerSubscriber):
        with self.lock:
            self.subscribers.append(subscriber)

    def getLocalMap(self):
        with self.lock:
            return self.localMap
    
    def getPositionCenteredGlobalMap(self):
        with self.lock:
            return self.positionCenteredGlobalMap



class MapServer(MapHandlerSubscriber, DerivedMapMakerSubscriber):

    @staticmethod
    def map_to_occupancy_grid(map: Map, frame):

       
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.frame_id = frame
        occupancy_grid.info.width = map.mapData.shape[1]
        occupancy_grid.info.height = map.mapData.shape[0]
        occupancy_grid.info.resolution = map.resolution
        occupancy_grid.info.origin.position.x = -map.origin[0] * map.resolution - map.resolution
        occupancy_grid.info.origin.position.y = -map.origin[1] * map.resolution - map.resolution
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.x = 0.0
        occupancy_grid.info.origin.orientation.y = 0.0
        occupancy_grid.info.origin.orientation.z = 0.0  
        occupancy_grid.info.origin.orientation.w = 1.0
        flattened_array = map.mapData.flatten()
        clamped_values = np.clip(flattened_array, -1, 100).astype(int)
        occupancy_grid.data = clamped_values.tolist()
        
        return occupancy_grid

    def __init__(self, rosNode, timePeriod, publishedTopicName, robot_map_frame, robot_base_frame):
        self.publisher = rosNode.create_publisher(OccupancyGrid, publishedTopicName, qos_profile=self.__get_qos_profile())
        self.publisherLocal = rosNode.create_publisher(OccupancyGrid, publishedTopicName+"Local", qos_profile=self.__get_qos_profile())
        self.publisherPositionCentered = rosNode.create_publisher(OccupancyGrid, publishedTopicName+"PositionCentered", qos_profile=self.__get_qos_profile())
        self.timer = rosNode.create_timer(timePeriod, self.__publish)
        self.robot_map_frame = robot_map_frame
        self.robot_base_frame = robot_base_frame
        self.lock = threading.Lock()
        self.map = None
        self.localMap = None
        self.positionCenteredGlobalMap = None

    def __get_qos_profile(self):
        qos_profile = QoSProfile(depth=10)  # Customize depth as needed
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        return qos_profile

    def __publish(self):
        with self.lock:
            if self.map:
                self.publisher.publish(self.map)
            if self.localMap:
                self.publisherLocal.publish(self.localMap)
            if self.positionCenteredGlobalMap:
                self.publisherPositionCentered.publish(self.positionCenteredGlobalMap)

    def updateGlobalMap(self, context : PointCountTrackMapHandler):
        mapData = self.map_to_occupancy_grid(context.getMap(), self.robot_map_frame)
        #localMapData = self.map_to_occupancy_grid(context.getLocalMap(), "base_link")
        with self.lock:
            self.map = mapData
    
    def updateDerivedMaps(self, context : DerivedMapMaker):
        localMap = self.map_to_occupancy_grid(context.getLocalMap(), self.robot_base_frame)
        positionCenteredGlobalMap = self.map_to_occupancy_grid(context.getPositionCenteredGlobalMap(), self.robot_map_frame)
        #localMapData = self.map_to_occupancy_grid(context.getLocalMap(), "base_link")
        with self.lock:
            self.localMap = localMap
            self.positionCenteredGlobalMap = positionCenteredGlobalMap


class MedianBlurMapPostProcessor(MapPostProcessor):
    
    def processMap(self, map: np.array) -> np.array:
        processedMap = np.zeros_like(map)
        mask = (map != -1)
        processedMap[mask] = map[mask]
        processedMap = processedMap.astype(np.uint8)
        kernel = np.ones((3, 3), np.uint8)
        processedMap = cv2.morphologyEx(processedMap, cv2.MORPH_CLOSE, kernel)
        processedMap = cv2.medianBlur(processedMap, ksize=3)
        processedMap = processedMap.astype(int)
        processedMap[~mask] = map[~mask]

        return processedMap


class NoneMapPostProcessor(MapPostProcessor):
    def processMap(self, map: np.array) -> np.array:
        return map


class GaussianBlurPostProcessor(MapPostProcessor):
    def processMap(self, map: np.array) -> np.array:
        minusOneFilter = map < 0

        image = map.copy()
        image[minusOneFilter] = 0
        image = image.astype(np.uint8)

        gray_image = image

        # Remove noise using Gaussian blur
        blurred_image = cv2.GaussianBlur(gray_image, (3, 3), 0)

        _, binary_image = cv2.threshold(blurred_image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # Fill holes using morphological operations
        kernel = np.ones((4, 4), np.uint8)
        filled_image = cv2.morphologyEx(blurred_image, cv2.MORPH_CLOSE, kernel)
        mask = filled_image > 100
        filled_image[mask] = 100
        filled_image = filled_image.astype(np.int8)
        filled_image[minusOneFilter] = -1

        filter = filled_image > 40
        filled_image[filter] = 100

        return filled_image


class ErodeMapPostProcessor(MapPostProcessor):
    def processMap(self, map: np.array) -> np.array:
        kernel = np.ones((2, 2), np.uint8)
        eroded_image = cv2.erode(map, kernel, iterations=1)
        return eroded_image


class RoadMap(Node):
    
    def __init__(self, config):
        super().__init__('RoadMap')

        global globalLogger
        globalLogger = self.get_logger()
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=config['tf_buffer']['cache_time_seconds']))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publishedTopic = config['topics']['published']
        self.pointCloudTopic = config['topics']['pointCloud']
        self.robot_base_frame = config['frames']['robot_base']
        self.robot_map_frame = config['frames']['robot_map']
        self.mapServerTimePeriod = config['mapServer']['timePeriod']

        mapHandlerParams = config['mapHandler']
        self.mapHandler = PointCountTrackMapHandler(mapHandlerParams['map_resolution'], mapHandlerParams['initial_map_height'], mapHandlerParams['initial_map_width'], mapHandlerParams['road_intensity_threshold'], NoneMapPostProcessor())

        pointCloudPostProcessorParams = config['pointCloudPostProcessor']['parameters']
        self.pointCloudPostProcessor = TransfromPointCloudPostProcessor(self.robot_map_frame, self.robot_base_frame, pointCloudPostProcessorParams[0], pointCloudPostProcessorParams[1])

        localMapHandlerParams = config['localMapHandler']['parameters']
        self.localMapHandler = DerivedMapMaker(self.tf_buffer, self.robot_map_frame, self.robot_base_frame, localMapHandlerParams[0])

        self.mapServer = MapServer(self, self.mapServerTimePeriod, self.publishedTopic, self.robot_map_frame, self.robot_base_frame)


        #self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=60))
        #self.tf_listener = TransformListener(self.tf_buffer, self)


        #self.publishedTopic = '/myRoad'
        #self.pointCloudTopic = '/oakd/points'
        #self.robot_base_frame = "base_link"
        #self.robot_map_frame = "map"
        #self.mapServerTimePeriod = 0.1
        

        #self.mapHandler = PointCountTrackMapHandler(0.035, 7, 12, 180, NoneMapPostProcessor())
        #self.pointCloudPostProcessor = TransfromPointCloudPostProcessor(self.robot_map_frame, self.robot_base_frame, 1.3, 0.6)



        #self.localMapHandler = DerivedMapMaker(self.tf_buffer, self.robot_map_frame, self.robot_base_frame, 2.0)

        #self.mapServer = MapServer(self, self.mapServerTimePeriod, self.publishedTopic, self.robot_map_frame, self.robot_base_frame)

        #self.localMapHandler.addSubscriber(self.mapServer )

        #self.mapHandler.addSubscriber(self.mapServer)
        #self.mapHandler.addSubscriber(self.localMapHandler)
        
        self.queue = queue.Queue(maxsize=3) 
        self.start_processing_pointcloud_queue()

        self.hold_for_transform()

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
        while not self.tf_buffer.can_transform(target_frame=self.robot_map_frame, source_frame=self.robot_base_frame, time=rclpy.time.Time()):
            self.get_logger().info("Transform " + str(self.robot_base_frame) + " to " + str(self.robot_map_frame) + " not available, retrying...")
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info("Transform " + str(self.robot_base_frame) + " to " + str(self.robot_map_frame) + " found")

   
    def point_cloud_callback(self, msg):
        try:
            self.queue.put_nowait(msg)  # Enqueue the message
        except:
            self.get_logger().info("pointcloud queue is full")


    def process_pointcloud_queue(self):
        while True:
            msg = self.queue.get()  # Dequeue the message
            self.add_to_map(msg)
    

    def start_processing_pointcloud_queue(self):
        processing_thread = threading.Thread(target=self.process_pointcloud_queue)
        processing_thread.start()


    def add_to_map(self, msg):        
        try:
            point_dtype = np.dtype([('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('intensity', 'f4')])
            pointcloud_data = np.frombuffer(msg.data, dtype=point_dtype)
            pointcloud_timestamp = msg.header.stamp
            s = time.time()
            points = self.pointCloudPostProcessor.getPoints(pointcloud_data, pointcloud_timestamp, msg.header.frame_id, self.tf_buffer)
            e = time.time()
            self.get_logger().info("points:" + str(e-s))
            s = time.time()
            self.mapHandler.addPoints(points)
            e = time.time()
            self.get_logger().info("map:" + str(e-s))
        except Exception as e:
           self.get_logger().info("error: " + str(e))
           return



def main(args=None):
    # Load the configuration from the JSON file
    with open('config.json') as f:
        config = json.load(f)

    rclpy.init(args=args)
    roadMap = RoadMap(config)
    try:
        rclpy.spin(roadMap)
    except KeyboardInterrupt:
        pass  # Allow clean shutdown on Ctrl+C
    finally:
        roadMap.destroy_node()
        rclpy.shutdown()




