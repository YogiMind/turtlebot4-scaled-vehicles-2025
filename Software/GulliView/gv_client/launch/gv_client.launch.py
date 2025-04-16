from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gv_client',
            executable='gv_socket_server',
            name='gv_socket_server_node',
            output='screen'
        ),
        Node(
            package='gv_client',
            executable='gv_socket_logger',
            name='gv_socket_logger_node',
            output='screen'
        ),
    ])