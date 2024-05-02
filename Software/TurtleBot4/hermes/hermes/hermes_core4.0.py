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
import os
from std_msgs.msg import String

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



class RoadLineSeparator(ABC):
    @abstractmethod
    def addPoints(self, points : np.array):
        pass


    def getRoadLinePoints(self) -> np.array:
        pass


    def getNonRoadLinePoints(self) -> np.array:
        pass



class Map():
    def __init__(self, mapData: np.array, origin: np.array, resolution: float, translation : np.array):
        self.mapData = mapData
        self.origin = origin
        self.resolution = resolution
        self.translation = translation
    
    def copy(self):
        return Map(self.mapData.copy(), self.origin.copy(), self.resolution, self.translation.copy())



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
    
    underlyingMapGrid = np.zeros((1, 1), dtype=np.int8)
    origin = np.array([0, 0], dtype=int)  # Origin point
    map = np.zeros((1, 1), dtype=np.int8)
    LockOne = threading.RLock()
    LockTwo = threading.RLock()
    subscribers = [MapHandlerSubscriber]
    

    def __init__(self, mapScale, initWitdth, initHeight, roadLineSeparator : RoadLineSeparator, mapPostProcessor: MapPostProcessor):
        self.mapScale = mapScale
        self.mapPostProcessor = mapPostProcessor
        self.roadLineSeparator = roadLineSeparator

        initHeight = initHeight / 4
        initWitdth = initWitdth / 4

        xy = np.round(np.array([[initHeight, initWitdth], [ -initHeight,-initWitdth]])[:,[0,1]]/self.mapScale).astype(int)

        self.__updateMapSize(xy)


    def __notifySubscribers(self):
        with self.LockOne:
            subscribers = self.subscribers
        for sub in subscribers:
            sub.updateGlobalMap(self)


    def __postProcessMap(self, map):
        return self.mapPostProcessor.processMap(map)


    def __formMap(self):
        map = self.underlyingMapGrid.copy()
        filter = map > 40
        map[filter] = 100
        return map


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

    def __updateUnderlyingMap(self, mappingPoints):
            #Scale points to map
            mappingPoints[:,[0,1]] = np.round(mappingPoints[:, [0,1]] / self.mapScale).astype(int)
           
            #Update map size if points outside current map
            xy = mappingPoints[:,[0,1]]
            self.__updateMapSize(xy)

            #Convert coordinates to map coords
            mappingPoints[:, [0,1]] += self.origin

            self.roadLineSeparator.addPoints(mappingPoints)
            road = self.roadLineSeparator.getRoadLinePoints()
            ground = self.roadLineSeparator.getNonRoadLinePoints()

            cellCounter = np.zeros_like(self.underlyingMapGrid)
            np.add.at(cellCounter, (road[:, 1].astype(int), road[:, 0].astype(int)), 4)
            np.add.at(cellCounter, (ground[:, 1].astype(int), ground[:, 0].astype(int)), -1)

            roadFilter = cellCounter > 0
            groundFilter = cellCounter < 0

            self.underlyingMapGrid[roadFilter] += 7
            self.underlyingMapGrid[groundFilter] -= 7

            mask = self.underlyingMapGrid < 0
            self.underlyingMapGrid[mask & groundFilter] = 0 #keep unknown cells at -1

            mask = self.underlyingMapGrid > 100
            self.underlyingMapGrid[mask] = 100


    def addSubscriber(self, subscriber: MapHandlerSubscriber):
        with self.LockOne:
            self.subscribers.append(subscriber)


    def getMap(self) -> Map:
        with self.LockOne:
            return Map(self.map, self.origin.copy(), self.mapScale, np.array([0,0]))
    

    def addPoints(self, points: np.array):
        with self.LockTwo:
            mappingPoints = np.array(points)
            self.__updateUnderlyingMap(mappingPoints)
            map = self.__formMap()
            map = self.__postProcessMap(map)
            with self.LockOne:
                self.map = map
            self.__notifySubscribers()



class ThresholdRoadLineSeparator(RoadLineSeparator):
    
    def __init__(self, roadIntensity) -> None:
        self.roadIntensity = roadIntensity
        self.roadLinePoints = None
        self.nonRoadLinePoints = None

    def addPoints(self, points: np.array):
            filter = points[:, 3] > self.roadIntensity            
            self.roadLinePoints = points[filter]
            self.nonRoadLinePoints = points[~filter]

    def getRoadLinePoints(self) -> np.array:
        return self.roadLinePoints
    
    def getNonRoadLinePoints(self) -> np.array:
        return self.nonRoadLinePoints



class NoneMapPostProcessor(MapPostProcessor):
    def processMap(self, map: np.array) -> np.array:
        return map



class RemoveSmallContoursPostProcessor(MapPostProcessor):
    def processMap(self, map: np.array) -> np.array:
        
        saveUnkown = map == -1
        map[saveUnkown] = 0
        map = map.astype(np.uint8)

        # Find contours
        contours, _ = cv2.findContours(map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Create a mask to store the filtered contours
        mask = np.zeros_like(map)
        
        # Iterate through contours
        for contour in contours:
            area = cv2.contourArea(contour)
            # If contour area is larger than min_area, draw it on the mask
            if area >= 5:
                cv2.drawContours(mask, [contour], -1, 255, thickness=cv2.FILLED)
        
        # Bitwise AND operation to keep only the filtered contours
        result = cv2.bitwise_and(map, mask)

        result = result.astype(np.int8)
        result[saveUnkown] = -1

        return result
        



class DerivedMapMaker():

    subscribers = [DerivedMapMakerSubscriber]

    def __init__(self, tfBuffer, robot_map_frame, robot_base_frame, mapSizeLocal, mapSizePositionCenteredGlobal, mapHandler : MapHandler, retransformTime):
        self.lock = threading.RLock()
        self.robot_map_frame = robot_map_frame
        self.robot_base_frame = robot_base_frame
        self.tfBuffer = tfBuffer
        self.mapHandler = mapHandler
        self.retransformTime = retransformTime

        self.mapSizeLocalMeters = mapSizeLocal
        self.mapSizePositionCenteredGlobal = mapSizePositionCenteredGlobal

        self.localMap = None
        self.positionCenteredGlobalMap = None

        processing_thread = threading.Thread(target=self.__retransformMaps)
        processing_thread.start()


    def __retransformMaps(self):
        global globalLogger
        while True:
            time.sleep(self.retransformTime)            
            mapData = self.mapHandler.getMap()
            self.map  = mapData
            self.localMapSize = np.ceil(self.mapSizeLocalMeters / mapData.resolution).astype(int)

            try:
                baseToMapTransform = self.tfBuffer.lookup_transform(target_frame=self.robot_map_frame, source_frame=self.robot_base_frame, time=rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=10))
                self.__formLocalMap(baseToMapTransform)
                self.__formPositionCenteredGlobalMap(baseToMapTransform)
                self.__notifySubscribers()
            except Exception as e:
                globalLogger.info("error: " + str(e))


    def __notifySubscribers(self):
        with self.lock:
            subscribers = self.subscribers
        
        for sub in subscribers:
            sub.updateDerivedMaps(self)


    @staticmethod
    def __select_square_around_center(arr, center, radius):
        size = radius * 2 + 1

        x, y = center
        half_size = size // 2
        
        # Calculate the indices of the square
        start_x = max(0, x - half_size)
        end_x = max(min(arr.shape[1], x + half_size + 1),0)
        start_y = max(0, y - half_size)
        end_y = max(min(arr.shape[0], y + half_size + 1),0)
        
        # Create the square
        square = np.full((size, size), -1)
        
        # Calculate the overlap of the square with the array
        arr_start_x = max(0, half_size - x)
        arr_end_x = max(min(size, half_size + (arr.shape[1] - x)),0)
        arr_start_y = max(0, half_size - y)
        arr_end_y = max(min(size, half_size + (arr.shape[0] - y)),0)
        
        # Fill the overlapped portion with values from the array
        square[arr_start_y:arr_end_y, arr_start_x:arr_end_x] = arr[start_y:end_y, start_x:end_x]
        
        return square


    def __formPositionCenteredGlobalMap(self, baseToMapTransform):
        translation = np.array([baseToMapTransform.transform.translation.x, baseToMapTransform.transform.translation.y])
        
        mapXYCenter = (translation / self.map.resolution).astype(int) + self.map.origin
        radius = int(self.mapSizePositionCenteredGlobal / self.map.resolution)
        
        map = self.__select_square_around_center(self.map.mapData, mapXYCenter, radius)

        with self.lock:
            self.positionCenteredGlobalMap = Map(map, np.array([radius, radius]), self.map.resolution, translation)


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
        
        with self.lock:
            self.localMap = Map(local_map_data, np.array([local_map_data.shape[1] //2, local_map_data.shape[0] //2]), self.map.resolution, np.array([0,0]))


    def addSubscriber(self, subscriber: MapHandlerSubscriber):
        with self.lock:
            self.subscribers.append(subscriber)


    def getLocalMap(self):
        with self.lock:
            return self.localMap
    

    def getPositionCenteredGlobalMap(self):
        with self.lock:
            return self.positionCenteredGlobalMap



class MapServer():

    @staticmethod
    def map_to_occupancy_grid(map: Map, frame):
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.frame_id = frame
        occupancy_grid.info.width = map.mapData.shape[1]
        occupancy_grid.info.height = map.mapData.shape[0]
        occupancy_grid.info.resolution = map.resolution
        occupancy_grid.info.origin.position.x = -map.origin[0] * map.resolution - map.resolution / 2 + map.translation[0]
        occupancy_grid.info.origin.position.y = -map.origin[1] * map.resolution - map.resolution / 2 + map.translation[1]
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.x = 0.0
        occupancy_grid.info.origin.orientation.y = 0.0
        occupancy_grid.info.origin.orientation.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0
        
        flattened_array = map.mapData.flatten()
        clamped_values = np.clip(flattened_array, -1, 100).astype(np.int8)
        occupancy_grid.data = clamped_values.tolist()

        return occupancy_grid

    def __init__(self, rosNode, timePeriod, publishedTopicName, robot_map_frame, robot_base_frame, mapHandler : MapHandler, derivedMapMaker : DerivedMapMaker):
        self.publisher = rosNode.create_publisher(OccupancyGrid, publishedTopicName, qos_profile=self.__get_qos_profile())
        self.publisherLocal = rosNode.create_publisher(OccupancyGrid, publishedTopicName+"Local", qos_profile=self.__get_qos_profile())
        self.publisherPositionCentered = rosNode.create_publisher(OccupancyGrid, publishedTopicName+"PositionCentered", qos_profile=self.__get_qos_profile())
        self.timer = rosNode.create_timer(timePeriod, self.__publish)
        self.robot_map_frame = robot_map_frame
        self.robot_base_frame = robot_base_frame

        self.mapHandler = mapHandler
        self.derivedMapMaker = derivedMapMaker

    def __get_qos_profile(self):
        qos_profile = QoSProfile(depth=10)  # Customize depth as needed
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        return qos_profile


    def __publish(self):
            try:
                map = self.map_to_occupancy_grid(self.mapHandler.getMap(), self.robot_map_frame)
                localMap = self.map_to_occupancy_grid(self.derivedMapMaker.getLocalMap(), self.robot_base_frame)
                positionCenteredGlobalMap = self.map_to_occupancy_grid(self.derivedMapMaker.getPositionCenteredGlobalMap(), self.robot_map_frame)
                self.publisher.publish(map)
                self.publisherLocal.publish(localMap)
                self.publisherPositionCentered.publish(positionCenteredGlobalMap)
            except:
                pass



class Hermes_mapper(Node):
    
    def __init__(self):
        super().__init__('Hermes_mapper')

        self.publishedTopic = '/myRoad'
        self.pointCloudTopic = '/oakd/points'
        self.robot_base_frame = "base_link"
        self.robot_map_frame = "map"
        self.mapServerTimePeriod = 0.35

        mapRes = 0.02
        initMapSize = 20

        pointCloudIntensityTrh = 190
        visionRangeForward = 1.0
        visionRangeSide = 0.55

        localMapSize = 1.6
        positionCenteredMapSize = 1.6

        global globalLogger
        globalLogger = self.get_logger()

        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=60))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.pointCloudPostProcessor = TransfromPointCloudPostProcessor(self.robot_map_frame, self.robot_base_frame, visionRangeForward, visionRangeSide)
        self.mapHandler = PointCountTrackMapHandler(mapRes, initMapSize, initMapSize, ThresholdRoadLineSeparator(pointCloudIntensityTrh), NoneMapPostProcessor())
        self.derivedMapMaker = DerivedMapMaker(self.tf_buffer, self.robot_map_frame, self.robot_base_frame, positionCenteredMapSize, localMapSize, self.mapHandler, self.mapServerTimePeriod)
        self.mapServer = MapServer(self, self.mapServerTimePeriod, self.publishedTopic, self.robot_map_frame, self.robot_base_frame, self.mapHandler, self.derivedMapMaker)

        
        self.queue = queue.Queue(maxsize=5) 
        self.start_processing_pointcloud_queue()

        self.hold_for_transform()

        self.subscriptionPointCloud = self.create_subscription(
            PointCloud2,
            self.pointCloudTopic,
            self.point_cloud_callback,
            rclpy.qos.QoSProfile(
                reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT, 
                depth=3
            )
        )

        self.subscriptionSaveMap = self.create_subscription(String, '/hermes_cmd', self.command, 10)
        


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
    

    #ros2 topic pub /hermes_cmd std_msgs/String "data: 'save'"
    #ros2 topic pub /hermes_cmd std_msgs/String "data: 'reset'"
    def command(self, msg):
        command = msg.data
        if(command == "save"):
            
            self.get_logger().info("svaving map")
            themap = self.mapHandler.getMap().mapData
            minusone = themap == -1
            themap[minusone] = 128
            

            inverted_image = 255 - (themap.astype(np.uint8) * 2.55).astype(np.uint8)
            # Create a 3-channel image from the greyscale image
            color_image = cv2.cvtColor(inverted_image, cv2.COLOR_GRAY2BGR)

            # Set pixels with value 128 (previously -1) to light blue
            light_blue = (230, 216, 173)  # bgr values for light blue
            color_image[minusone] = light_blue

            home_dir = os.path.expanduser("~")
            # Save the color image
            cv2.imwrite(os.path.join(home_dir, 'the_output_map.png'), color_image)
            self.get_logger().info("map saved")
        elif(command == "reset"):
            self.mapHandler.underlyingMapGrid = np.zeros((1, 1), dtype=np.int8)
            

def main(args=None):
    rclpy.init(args=args)
    hermes = Hermes_mapper()
    try:
        rclpy.spin(hermes)
    except KeyboardInterrupt:
        pass
    finally:
        hermes.destroy_node()
        rclpy.shutdown()




