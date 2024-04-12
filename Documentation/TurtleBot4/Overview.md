# TurtleBot 4 Overview

# Introduction

The TurtleBot 4 consists of an iRobot [Create 3](../TurtleBot4/On-Board-Computers/Create3/) base which has been extended with a [Raspberry Pi 4B (4 GB)](../TurtleBot4/On-Board-Computers/Raspberry-Pi/), a User Interface PCBA which allow extra sensors to be added through the 4 USB-C ports (requires a change of cable), an [OAK-D Pro](../TurtleBot4/Sensors/Camera-OAK-D.md) camera and a [RP-LiDAR](../TurtleBot4/Sensors/LiDAR.md). 

The TurtleBot 4 runs on ROS2 (in our case ROS2 Humble) and 


| Specification           | Value                                                                                       |
|-------------------------|---------------------------------------------------------------------------------------------|
| External Dimensions     | 341 x 339 x 351 mm (13.4 x 13.3 x 13.8 in)                                                  |
| Weight                  | 3.9 kg (8.6 lbs)                                                                            |
| Wheels (Diameter)       | 72 mm (2.83 in)                                                                             |
| Ground Clearance        | 4.5 mm (0.17 in)                                                                            |
| Max Payload             | 9 Kg - Default, 15 kg - Custom Configuration                                                |
| Max Speed               | 0.31 m/s (safe mode), 0.46 m/s (cliff sensors disabled)                                     |
| Max Rotational Speed    | 1.90 rad/s                                                                                  |
| Battery Chemistry       | 26 Wh Lithium Ion (14.4V nominal) Rechargeable                                              |
| Charge Time             | 2.5 hrs                                                                                     |
| Operating Time          | 2.5 - 4.0 hrs (load dependent)                                                              |
| User Power              | VBAT @ 300mA, 12V @300mA, 5V @ 500mA 3.3V @ 250mA                                           |
| Docking                 | Base station for docked charging                                                            |
| 2D LIDAR                | RPLIDAR-A1 0.15-12m range; 8kHz sampling rate; 360 degree angular range; 1 degree resolution|
| Camera                  | OAK-D-PRO 4K RGB auto focus camera (IMX378), Mono stereo camera pair (OV9282)              |
|                         | IMU, Spatial AI processor, IR Laser Dot Projector & Illumination LED                        |
| Other Sensors           | 2x front bumper zones, 2x wheel encoders, 4x IR cliff sensors, 6x IR obstacle sensors,      |
|                         | 1x downward optical flow sensor for odometry, 1x 3D gyroscope, 1x 3D accelerometer,         |
|                         | 1x battery level monitor                                                                    |
| Other Actuators         | 2x Drive Motors, 6x RGB LED Ring, 1x Speaker, 5 Status LEDs,                                |
|                         | 2 User LEDs, 128x64 OLED Display                                                            |
| Computer                | Raspberry Pi 4B (4 GB)                                                                      |
| Software                | Ubuntu 20.04, ROS 2                                                                         |
