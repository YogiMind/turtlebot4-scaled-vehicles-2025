from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes, Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
#    pkg_turtlebot4_bringup = get_package_share_directory('turtlebot4_bringup')
    this_pkg_share = get_package_share_directory('road_navigate')



    ARGUMENTS = [
        DeclareLaunchArgument('camera', default_value='oakd_pro'),
        DeclareLaunchArgument('params_file', default_value=[PathJoinSubstitution([this_pkg_share, 'config', 'pc']), '.yaml']),
        DeclareLaunchArgument('namespace', default_value='',description='Robot namespace')
    ]

    
    node3 = Node(
        package='road_navigate', 
        executable='capture_road', 
        name='capture_road', 
        output='screen')


    node2 = LoadComposableNodes(
            target_container='oakd_container',
            composable_node_descriptions=[
                    ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyziNode',
                    name='point_cloud_xyzi',
                    remappings=[('depth/image_rect', '/theLinesDepth'),
                                ('intensity/image_rect', 'oakd/right/image_rect'),
                                ('intensity/camera_info', 'oakd/stereo/camera_info'),
                                ('points', 'oakd/points')
                                ]),
            ],
        )
    

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(node3)
    ld.add_action(node2)
    return ld
