import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import (ComposableNodeContainer, LoadComposableNodes,
                                Node, PushRosNamespace)
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Declare the namespace as a launch argument so it can be set from command line
    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="", description="Namespace for the robot"
    )

    namespace = LaunchConfiguration("namespace")

    tb4_bringup = get_package_share_directory("turtlebot4_navigation")
    ourPackage = get_package_share_directory("hermes")

    slamPath = os.path.join(tb4_bringup, "launch", "slam.launch.py")

    pathToSlamPara = os.path.join(ourPackage, "config", "slam.yaml")

    # namespaced_pathToSlamPara = RewrittenYaml(
    #    source_file=pathToSlamPara,
    #    root_key=namespace,
    #    param_rewrites={},
    #    convert_types=True)

    # Include SLAM launch file
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slamPath),
        launch_arguments={"params": pathToSlamPara}.items(),
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
                    ("/tf", "/glenn/tf"),
                    ("/tf_static", "/glenn/tf_static"),
                    ("/theDepth", "/glenn/theDepth"),
                    ("/theIntensity", "/glenn/theIntensity"),
                    ("/oakd/stereo/camera_info", "/glenn/oakd/stereo/camera_info"),
                    ("/oakd/points", "/glenn/oakd/points"),
                ],
            ),
            Node(
                package="hermes",
                executable="depth_intensity_image_syncer",
                name="depth_intensity_image_syncer",
                output="screen",
                remappings=[
                    ("/theDepth", "/glenn/theDepth"),
                    ("/theIntensity", "/glenn/theIntensity"),
                    ("/oakd/right/image_rect", "/glenn/oakd/right/image_rect"),
                    ("/oakd/stereo/image_raw", "/glenn/oakd/stereo/image_raw"),
                ],
            ),
            ComposableNodeContainer(
                name="oakd_container_pc",
                namespace=namespace,  # Will be overwritten by PushRosNamespace
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

    ld = LaunchDescription()
    ld.add_action(namespace_arg)
    ld.add_action(grouped_nodes)
    ld.add_action(slam)
    return ld
