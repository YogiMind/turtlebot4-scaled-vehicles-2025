import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the robot'
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_localization_node',
            namespace=namespace,
            output='screen',
            parameters=[PathJoinSubstitution([get_package_share_directory("hermes"), "config", "ekf.yaml"])],
            remappings=[
                ('/odom', [namespace, '/odom']),
                ('/gv_pose', [namespace, '/gv_pose'])
            ]
        )
    ])
