# depth_intensity_image_syncer.py - ROS2 Node for Synchronizing Depth and Intensity Images

This ROS2 node, `depth_intensity_image_syncer.py`, synchronizes and processes images from the `/oakd/right/image_rect` (right rectified image) and `/oakd/stereo/image_raw` (depth image) topics. It publishes the synchronized images to the `/theIntensity` (right image) and `/theDepth` (depth image) topics.

Link to code: [depth_intensity_image_syncer.py](../../Software/TurtleBot4/hermes/hermes/depth_intensity_image_syncer.py)

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Classes](#classes)
- [Functions](#functions)
- [Main Execution](#main-execution)

## Features

- Subscribes to `/oakd/right/image_rect` (right rectified image) and `/oakd/stereo/image_raw` (depth image) topics.
- Synchronizes images based on their timestamps (within a 0.07-second tolerance).
- Publishes synchronized images to `/theIntensity` (right image) and `/theDepth` (depth image) topics.
- Uses threading and queues for efficient processing.

## Installation

1. **Prerequisites:**
   - ROS2 installed
   - `sensor_msgs`, `builtin_interfaces`, and `cv_bridge` packages installed
   - OpenCV Python (`cv2`) installed
2. **Building:**
   - Create a ROS2 package for this node.
   - Place the `depth_intensity_image_syncer.py` file within the package's `scripts` directory.
   - Build the package using `colcon build`.

## Usage

1. **Launching:**
   - Ensure your OAK-D camera is publishing images to the `/oakd/right/image_rect` and `/oakd/stereo/image_raw` topics.
   - Run the node: `ros2 run <your_package_name> depth_intensity_image_syncer`
2. **Subscribed Topics:**
   - `/oakd/right/image_rect`: Right rectified image from the OAK-D camera.
   - `/oakd/stereo/image_raw`: Depth image from the OAK-D camera.
3. **Published Topics:**
   - `/theIntensity`: Synchronized right rectified image.
   - `/theDepth`: Synchronized depth image.

## Classes

- `SyncedImages`:
    - Data class to hold a pair of synchronized depth and right images.
- `CaptureRoad(Node)`:
    - The main ROS2 node class.
    - Subscribes to image topics, synchronizes images, and publishes them.

## Functions

- `get_timestamp_seconds(msg)`:
    - Extracts the timestamp in seconds from a ROS2 message header.

## Main Execution

The `main` function initializes ROS2, creates an instance of the `CaptureRoad` node, and spins the node to process incoming messages and execute the image synchronization logic.

**Additional Notes**

- The synchronization tolerance of 0.07 seconds is hardcoded in the `sync_messages` function. You might want to make it configurable if you need different tolerances.
- If the image queues become full, the node logs a message but doesn't discard older images. Consider adding logic to drop older images in such cases to maintain responsiveness. 
- The code does not currently perform any processing on the images beyond synchronization. If you need additional image processing or analysis, you can modify the `process_synced_messages` function.
