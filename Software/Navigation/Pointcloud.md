# Pointcloud

## Overview

## Hardware
Basically, most [Sensors](../../Documentation/TurtleBot4/Sensors/) on the Turtlebot4 is used for mapping. The [OAK-D Camera](../../Documentation/TurtleBot4/Sensors/Camera-OAK-D.md) uses its depth camera, [IMU's](../../Documentation/TurtleBot4/Sensors/Inertial%20Measurement-IMU.md) are used to help determine the angle and position the Turtlebot is moving while the [LiDAR](../../Documentation/TurtleBot4/Sensors/LiDAR.md) detect walls and potential physical obstacles. 

[Mapping](#mapping) is also depending on contrasted tape (white tape on dark floor) that is used in our lab environment. We recommend using thick enough lines as it may greatly increase accuracy as illustrated in the image below.  
<img src="../../Documentation/Assets/Images/Turtlebot4/Software/sc_added_tape_vs_no_added_tape_March20th.png" width="300">  
*The lower line (2 rows of white tape) is significantly more visible than the upper one (1 row of white tape)*

## Software
[RViz](../../Documentation/TurtleBot4/Software/RViz/RViz.md) is used for visualisation and interpreting sensor data. 

### Mapping
For mapping, [pointcloud_map.py](../../Software/TurtleBot4/road_navigate/road_navigate/pointcloud_map.py) is used. The code takes data from the depth sensor of the [OAK-D Camera](../../Documentation/TurtleBot4/Sensors/Camera-OAK-D.md) which is used to visualize the roads on the ground (white tape on dark floor).  

<img src="../../Documentation/Assets/Images/Turtlebot4/Software/RViz_road_detection_v1.png" width="500">  

*First iteration of road detection using depth data*  

[pointcloud_map.py](../../Software/TurtleBot4/road_navigate/road_navigate/pointcloud_map.py) implements [Numba's no python mode(external link)](https://numba.readthedocs.io/en/stable/glossary.html#term-nopython-mode) to increase the efficiency of the code by compiling certain functions without accessing Python's own C API. 
the ```collect_points_into_grid```function takes in a `data_array` which should hold `x,y,z,intensity` values. `x,y,z` is of course coordinates, while `intensity` represent the color intensity (black and white only) of the coordinate. 


### Configuration
