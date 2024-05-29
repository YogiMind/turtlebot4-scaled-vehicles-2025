# Hermes_mapper

This ROS2 node generates a map based on point cloud data from a sensor (e.g., an OAK-D camera). It processes the point cloud, creates a map representation, and publishes it as an OccupancyGrid message. The map is built incrementally by accumulating point cloud data over time.

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
- [Classes](#classes)
- [Functions](#functions)
- [Main Execution](#main-execution)

## Features

- Processes point cloud data from a ROS2 topic (e.g., `/oakd/points`)
- Transforms point cloud data from camera frame to base_link and map frames.
- Filters points based on intensity and range.
- Separates points into road and non-road categories.
- Generates a global map by accumulating point cloud data.
- Creates local and position-centered global maps.
- Publishes maps as OccupancyGrid messages on topics like `/myRoad`, `/myRoadLocal`, `/myRoadPositionCentered`.
- Saves the map as an image (`the_output_map.png`) in the user's home directory when triggered by a ROS2 message on the `/hermes_cmd` topic.
- Resets the map when triggered by a ROS2 message on the `/hermes_cmd` topic.

## Installation

1. **Prerequisites:**
   - ROS2 installed
   - `sensor_msgs`, `nav_msgs`, `tf2_ros`, `std_msgs` packages
   - `numpy`, `opencv-python` (cv2), `numba` libraries
2. **Building:**
   - Create a ROS2 package for this node.
   - Place this code in a Python file (e.g., `hermes_mapper.py`) within the package's `scripts` directory.
   - Build the package using `colcon build`.

## Configuration

Parameters are configured directly in the `Hermes_mapper` class constructor:

- `publishedTopic`: ROS2 topic to publish the map on (default: `/myRoad`)
- `pointCloudTopic`: ROS2 topic to subscribe to for point cloud data (default: `/oakd/points`)
- `robot_base_frame`: Robot's base frame (default: `"base_link"`)
- `robot_map_frame`:  Map frame (default: `"map"`)
- `mapServerTimePeriod`: Map publishing time period in seconds (default: 0.35)
- Other parameters for map resolution, intensity threshold, vision range, etc., can be adjusted in the constructor.

## Usage

1. **Launching:**
   - After building, run the node using `ros2 run <your_package_name> Hermes_mapper`
2. **Saving the map:**
   - Publish a message with the string `'save'` to the `/hermes_cmd` topic.
3. **Resetting the map:**
   - Publish a message with the string `'reset'` to the `/hermes_cmd` topic.

## Classes

- `PointCloudPostProcessor` (Abstract Base Class):
    - Defines an interface for post-processing point cloud data.
- `MapHandlerSubscriber` (ABC):
    - Interface for subscribers to be notified when the global map is updated.
- `DerivedMapMakerSubscriber` (ABC):
    - Interface for subscribers to be notified when derived maps are updated.
- `MapPostProcessor` (ABC):
    - Interface for post-processing the map.
- `RoadLineSeparator` (ABC):
    - Interface for separating road and non-road points.
- `Map`:
    - Represents a 2D map with data, origin, resolution, and translation.
- `MapHandler` (ABC):
    - Interface for handling the map data and subscribers.
- `TransfromPointCloudPostProcessor`:
    - Implements `PointCloudPostProcessor` to transform point cloud data between frames and filter points.
- `PointCountTrackMapHandler`:
    - Implements `MapHandler` to build and manage the map using point counts.
- `ThresholdRoadLineSeparator`:
    - Implements `RoadLineSeparator` to separate road points based on intensity. 
- `NoneMapPostProcessor`:
    - Implements `MapPostProcessor` and does nothing to the map.
- `RemoveSmallContoursPostProcessor`:
    - Implements `MapPostProcessor` to remove small contours from the map.
- `DerivedMapMaker`:
    - Creates and maintains local and position-centered global maps based on the main map.
- `MapServer`:
    - Publishes the different map representations as OccupancyGrid messages.
- `Hermes_mapper`:
    - The main ROS2 node class that coordinates the mapping process.

## Functions

- `transform_points(points, transform)`: Transforms 3D points using a given transformation.
- `map_to_occupancy_grid(map, frame)`: Converts a `Map` object into an `OccupancyGrid` ROS message.

## Main Execution

The `main` function initializes the ROS2 node, creates an instance of the `Hermes_mapper` class, and then enters the ROS2 spin loop to process incoming messages and perform the mapping logic.
