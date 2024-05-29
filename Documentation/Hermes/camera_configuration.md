# Hermes: Camera Configuration for TurtleBot 4

This guide provides instructions on how to modify the camera configuration for the TurtleBot 4 within the "Hermes" system. 

## Camera Position and Rotation Parameters

The following parameters within the TurtleBot 4 URDF file (`/opt/ros/humble/share/turtlebot4_description/urdf/standard/turtlebot4.urdf.xacro`) control the camera's position and orientation:

- **`camera_mount_x_offset`:** Horizontal offset (X-axis) of the camera mount.
- **`camera_mount_y_offset`:** Vertical offset (Y-axis) of the camera mount.
- **`camera_mount_z_offset`:** Distance from the base (Z-axis) of the camera mount.
- **`oakd_pro_x_offset`:** Horizontal offset (X-axis) of the OAK-D Pro camera from the camera mount.
- **`oakd_pro_y_offset`:** Vertical offset (Y-axis) of the OAK-D Pro camera from the camera mount.
- **`oakd_pro_z_offset`:** Distance from the camera mount (Z-axis) of the OAK-D Pro camera.
- **`rpy` (within the `<origin>` tag):** Roll, pitch, and yaw angles of the OAK-D Pro camera (in radians).

**Note:** The current configuration uses values of `0.9` for pitch and `0` for roll and yaw.

## Modification Steps

1. **Locate the URDF file:**
   - Open the file `/opt/ros/humble/share/turtlebot4_description/urdf/standard/turtlebot4.urdf.xacro` in a text editor.

2. **Adjust the parameters:**
   - Identify the `<xacro:property>` lines for the offsets (`camera_mount_*_offset`, `oakd_pro_*_offset`).
   - Modify the `value` attributes to match your desired camera position and orientation.
   - Consider the following:
      - **Positive X values:** Move the camera or mount to the right.
      - **Negative X values:** Move the camera or mount to the left.
      - **Positive Y values:** Move the camera or mount up.
      - **Negative Y values:** Move the camera or mount down.
      - **Positive Z values:** Move the camera or mount forward.
      - **Negative Z values:** Move the camera or mount backward.

3. **Modify rotation (if needed):**
   - Find the `<origin>` tag within the `<xacro:oakd>` block.
   - Adjust the `rpy` values as necessary (in radians).

4. **Save and reload:**
   - Save the modified URDF file.
   - If your TurtleBot 4 is currently running, restart the relevant ROS nodes to apply the changes.

## Example: Camera Facing Forward and Downward

```xml
<xacro:property name="camera_mount_x_offset" value="0.0"/> 
<xacro:property name="camera_mount_y_offset" value="0.0"/>
<xacro:property name="camera_mount_z_offset" value="0.15"/> 
<xacro:property name="oakd_pro_x_offset" value="0.0"/>
<xacro:property name="oakd_pro_y_offset" value="0.0"/>
<xacro:property name="oakd_pro_z_offset" value="0.05"/> 
<xacro:oakd model="pro" parent_link="oakd_camera_bracket">
  <origin xyz="${oakd_pro_x_offset} ${oakd_pro_y_offset} ${oakd_pro_z_offset}" rpy="0 0.5 0"/> 
</xacro:oakd>
