


Launch the following launch files to start hermes.

1. start the depthCam.launch.py on the robot via "ros2 launch hermes depthCam.launch.py"
2. Then launch start_hermes.launch.py on a computer via "ros2 launch hermes start_hermes.launch.py"
3. To get navigation, launch nav2.launch.py via "ros2 launch hermes nav2.launch.py", we did this on a computer, but it might work on the robot as well.





# depthCam.launch.py - ROS2 Launch File for OAK-D Camera

This ROS2 launch file, `depthCam.launch.py`, initializes and configures the Oak-D camera driver to capture depth and RGB images.

Link to code: [depthCam.launch.py](../../Software/TurtleBot4/hermes/launch/depthCam.launch.py)

## Table of Contents

- [Purpose](#purpose)
- [Launch Arguments](#launch-arguments)
- [Technical Details](#technical-details)

## Purpose

* Loads camera parameters from a YAML configuration file.
* Creates a composable node container to run the `depthai_ros_driver` camera node.

## Launch Arguments

* `params_file`:
    - Default: `hermes/config/pc.yaml`
    - Description: Path to a YAML file containing camera-specific settings.
* `namespace`:
    - Default: (empty string)
    - Description: Optional namespace for the camera node.

## Technical Details

1. **Parameter Loading:**
   - The launch file loads the camera parameters from the specified YAML file or the default configuration.

2. **Composable Node Container:**
   - A composable node container named `oakd_container` is created.
   - It uses the `rclcpp_components` package and `component_container` executable.

3. **Camera Node:**
   - The container includes a composable node for the OAK-D camera using the `depthai_ros_driver::Camera` plugin.
   - The loaded parameters are passed to the camera node.

4. **Output:**
   - The launch file outputs logs and messages to the screen.


# nav2.launch.py - ROS2 Launch File for Nav2 Navigation Stack

This ROS2 launch file, `nav2.launch.py`, starts and configures the Nav2 navigation stack for robot navigation tasks.

Link to code: [nav2.launch.py](../../Software/TurtleBot4/hermes/launch/nav2.launch.py)

## Table of Contents

- [Purpose](#purpose)
- [Launch Arguments](#launch-arguments)
- [Technical Details](#technical-details)

## Purpose

* Launches the Nav2 navigation stack with specified parameters.
* Sets up remappings for scan topics within the specified namespace.

## Launch Arguments

* `use_sim_time`: 
    - Default: `false`
    - Description: Whether to use simulated time (boolean).
* `params_file`:
    - Default: `hermes/config/nav2.yaml`
    - Description: Path to a YAML file containing Nav2 parameters.
* `namespace`:
    - Default: (empty string)
    - Description: Namespace for the Nav2 nodes.

## Technical Details

1. **Namespace Handling:**
   - Ensures the namespace starts with a forward slash (`/`) if provided.

2. **Nav2 Launch Inclusion:**
   - Includes the `navigation_launch.py` file from the `nav2_bringup` package.

3. **Topic Remapping:**
   - Remaps the `/global_costmap/scan` and `/local_costmap/scan` topics to `/scan` within the specified namespace.

4. **Launch Arguments:**
   - Passes the `use_sim_time`, `params_file`, `use_composition` (set to 'False'), and `namespace` arguments to the included Nav2 launch file.


# start_hermes.launch.py - ROS2 Launch File for Hermes Robot

This ROS2 launch file, `start_hermes.launch.py`, orchestrates the startup of various nodes.

Link to code: [start_hermes.launch.py](../../Software/TurtleBot4/hermes/launch/start_hermes.launch.py)

## Table of Contents

- [Purpose](#purpose)
- [Included Launch Files and Nodes](#included-launch-files-and-nodes)

## Purpose

* Launches SLAM (Simultaneous Localization and Mapping).
* Starts core Hermes functionality (`hermes_core` node).
* Synchronizes depth and intensity images.
* Processes point cloud data.

## Included Launch Files and Nodes

1. **SLAM Launch:**
   - Includes `slam.launch.py` from the `turtlebot4_navigation` package.
   - Uses parameters from `hermes/config/slam.yaml`.

2. **Hermes Core Node:**
   - Starts the `hermes_core` executable from the `hermes` package.

3. **Image Syncer Node:**
   - Starts the `depth_intensity_image_syncer` executable from the `hermes` package.

4. **Point Cloud Node:**
   - Creates a composable node container running the `PointCloudXyziNode` from the `depth_image_proc` package.
   - Remaps topics for depth and intensity images and camera info.
