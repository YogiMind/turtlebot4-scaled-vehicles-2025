import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import (ComposableNodeContainer, LoadComposableNodes,
                                Node, PushRosNamespace)
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

ARGUMENTS = [
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        choices=["true", "false"],
        description="Use sim time",
    ),
    DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [get_package_share_directory("hermes"), "config", "slam.yaml"]
        ),
        description="Nav2 parameters",
    ),
    DeclareLaunchArgument("namespace", default_value="", description="Robot namespace"),
]


def launch_setup(context, *args, **kwargs):

    namespace = LaunchConfiguration("namespace")
    slam_params = LaunchConfiguration("params_file")

    namespace_str = namespace.perform(context)
    if namespace_str and not namespace_str.startswith("/"):
        namespace_str = "/" + namespace_str

    tb4_navigation = get_package_share_directory("turtlebot4_navigation")

    launch_slam = os.path.join(tb4_navigation, "launch", "slam.launch.py")

    # Include SLAM launch file
    slam = (
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_slam),
            launch_arguments={"params": slam_params}.items(),
        )
    )

    grouped_nodes = GroupAction(
        [
            # Push the namespace for all nodes in this group
            PushRosNamespace(namespace),
            Node(
                package="hermes",
                executable="hermes_core",
                name="hermes_core",
                output="screen",
                remappings=[
                    ("/tf", namespace_str + "/tf"),
                    ("/tf_static", namespace_str + "/tf_static"),
                    ("/theDepth", namespace_str + "/theDepth"),
                    ("/theIntensity", namespace_str + "/theIntensity"),
                    (
                        "/oakd/stereo/camera_info",
                        namespace_str + "/oakd/stereo/camera_info",
                    ),
                    ("/oakd/points", namespace_str + "/oakd/points"),
                ],
            ),
            Node(
                package="hermes",
                executable="depth_intensity_image_syncer",
                name="depth_intensity_image_syncer",
                output="screen",
                remappings=[
                    ("/theDepth", namespace_str + "/theDepth"),
                    ("/theIntensity", namespace_str + "/theIntensity"),
                    (
                        "/oakd/right/image_rect",
                        namespace_str + "/oakd/right/image_rect",
                    ),
                    (
                        "/oakd/stereo/image_raw",
                        namespace_str + "/oakd/stereo/image_raw",
                    ),
                ],
            ),
            ComposableNodeContainer(
                name="oakd_container_pc",
                namespace="",  # Will be overwritten by PushRosNamespace
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="depth_image_proc",
                        plugin="depth_image_proc::PointCloudXyziNode",
                        name="point_cloud_xyzi",
                        remappings=[
                            ("depth/image_rect", "theDepth"),
                            ("intensity/image_rect", "theIntensity"),
                            ("intensity/camera_info", "oakd/stereo/camera_info"),
                            ("points", "oakd/points"),
                        ],
                    ),
                ],
                output="screen",
            ),
        ]
    )
    return [slam, grouped_nodes]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
