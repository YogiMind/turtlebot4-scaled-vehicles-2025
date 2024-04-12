
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes, Node
from nav2_common.launch import RewrittenYaml
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os


def generate_launch_description():

    tb4_bringup = get_package_share_directory('turtlebot4_navigation')
    ourPackage = get_package_share_directory('hermes')

    slamPath = os.path.join(tb4_bringup, "launch", "slam.launch.py")

    pathToSlamPara = os.path.join(ourPackage, "config", "slam.yaml")

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slamPath),
        launch_arguments={'params': pathToSlamPara}.items(),
        )


    node3 = Node(
        package='hermes', 
        executable='hermes_core', 
        name='hermes_core',
        output='screen'
        )
    
    node1 = Node(
        package='hermes', 
        executable='depth_intensity_image_syncer', 
        name='depth_intensity_image_syncer', 
        output='screen'
        )
     

    node2 = ComposableNodeContainer(
            name='oakd_container_pc',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                    ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyziNode',
                    name='point_cloud_xyzi',
                    remappings=[('depth/image_rect', 'theDepth'),
                                ('intensity/image_rect', 'theIntensity'),
                                ('intensity/camera_info', 'oakd/stereo/camera_info'),
                                ('points', 'oakd/points')
                                ]),
            ],
            output='screen',
        )

    ld = LaunchDescription()
    ld.add_action(node1)
    ld.add_action(slam)
    ld.add_action(node2)
    ld.add_action(node3)
    return ld
