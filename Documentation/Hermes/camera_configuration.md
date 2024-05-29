# Hermes: Camera Configuration for TurtleBot 4

This guide provides instructions on how to modify the camera configuration for the TurtleBot 4 within the "Hermes" system. 

## Camera Position and Rotation Parameters

The following parameters within the TurtleBot 4 URDF file (`/opt/ros/humble/share/turtlebot4_description/urdf/standard/turtlebot4.urdf.xacro`) (on the robot) control the camera's position and orientation, following ROS2 REP-103 conventions:

- **`camera_mount_x_offset`:** Forward offset (X-axis) of the camera mount in meters.
- **`camera_mount_y_offset`:** Left offset (Y-axis) of the camera mount in meters.
- **`camera_mount_z_offset`:** Upward offset (Z-axis) of the camera mount in meters.
- **`oakd_pro_x_offset`:** Forward offset (X-axis) of the OAK-D Pro camera from the camera mount in meters.
- **`oakd_pro_y_offset`:** Left offset (Y-axis) of the OAK-D Pro camera from the camera mount in meters.
- **`oakd_pro_z_offset`:** Upward offset (Z-axis) of the OAK-D Pro camera from the camera mount in meters.
- **`rpy` (within the `<origin>` tag):** Roll, pitch, and yaw angles of the OAK-D Pro camera (in radians).

**Note:** The current configuration uses values of `0.9` for pitch and `0` for roll and yaw. This is consistent with ROS 2 conventions where positive rotation about the Y-axis (pitch) results in a downward tilt.

## Modification Steps

1. **Locate the URDF file:**
   - Open the file `/opt/ros/humble/share/turtlebot4_description/urdf/standard/turtlebot4.urdf.xacro` in a text editor.

2. **Adjust the parameters:**
   - Identify the `<xacro:property>` lines for the offsets (`camera_mount_*_offset`, `oakd_pro_*_offset`).
   - Modify the `value` attributes to match your desired camera position and orientation. Ensure values are in meters for offsets.
   - Consider the following (based on REP-103):
      - **Positive X values:** Move the camera or mount forward.
      - **Negative X values:** Move the camera or mount backward.
      - **Positive Y values:** Move the camera or mount to the left.
      - **Negative Y values:** Move the camera or mount to the right.
      - **Positive Z values:** Move the camera or mount upward.
      - **Negative Z values:** Move the camera or mount downward.

3. **Modify rotation (if needed):**
   - Find the `<origin>` tag within the `<xacro:oakd>` block.
   - Adjust the `rpy` values as necessary (in radians). Adhere to the right-hand rule for positive rotation directions.

4. **Save and reload:**
   - Save the modified URDF file.
   - Restart the robot. If changes dont apply, reinstall the robot upstart script in Turtlebot4-setup on the robot.  In terminal write Turtlebot4-setup then go to ros setup then go to robot upstart, then unisntsall and reinstall.

## Example: Camera Facing Forward and Downward

```xml
<xacro:property name="camera_mount_x_offset" value="0.15"/> 
<xacro:property name="camera_mount_y_offset" value="0.0"/>
<xacro:property name="camera_mount_z_offset" value="0.0"/> 
<xacro:property name="oakd_pro_x_offset" value="0.05"/>
<xacro:property name="oakd_pro_y_offset" value="0.0"/>
<xacro:property name="oakd_pro_z_offset" value="0.0"/> 
<xacro:oakd model="pro" parent_link="oakd_camera_bracket">
  <origin xyz="${oakd_pro_x_offset} ${oakd_pro_y_offset} ${oakd_pro_z_offset}" rpy="0 0.5 0"/> 
</xacro:oakd>
```

## Further reading
[ROS 2 - Standard Units of Measure and Coordinate Conventions](https://www.ros.org/reps/rep-0103.html)