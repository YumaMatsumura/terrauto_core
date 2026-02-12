#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    bringup_share = get_package_share_directory("terrauto_bringup")

    localization_component_params_path = os.path.join(
        bringup_share, "params", "localization_component_params.yaml"
    )

    # LaunchConfigurations
    localization_component_params_file = LaunchConfiguration("localization_component_params_file")
    use_composition = LaunchConfiguration("use_composition")
    use_multithread = LaunchConfiguration("use_multithread")
    container_executable = LaunchConfiguration("container_executable")

    # Declare args
    declare_localization_component_params_file_cmd = DeclareLaunchArgument(
        "localization_component_params_file",
        default_value=localization_component_params_path,
        description="Full path to the ROS 2 parameters file for localization component nodes",
    )
    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed nodes",
    )
    declare_use_multithread_cmd = DeclareLaunchArgument(
        "use_multithread",
        default_value="False",
        description="Use MultiThreadedExecutor for component container (component_container_mt)",
    )

    # Decide container executable
    set_container_executable_single = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(use_multithread),
    )
    set_container_executable_multi = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(use_multithread),
    )

    # Load nodes
    load_composition_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            Node(
                name="terrauto_localization_component_container",
                package="rclcpp_components",
                executable=container_executable,
                output="screen",
            ),
            LoadComposableNodes(
                target_container="terrauto_localization_component_container",
                composable_node_descriptions=[
                    ComposableNode(
                        name="ndt_scan_matcher_node",
                        package="terrauto_ndt_scan_matcher",
                        plugin="terrauto_ndt_scan_matcher::NdtScanMatcher",
                        parameters=[
                            localization_component_params_file,
                        ],
                        remappings=[
                            ("points", "/voxel_grid_filter/points"),
                        ],
                        extra_arguments=[{"use_intra_process_comms": False}],
                    ),
                ],
            ),
        ],
    )

    load_nodes = GroupAction(
        condition=UnlessCondition(use_composition),
        actions=[
            Node(
                name="ndt_scan_matcher_node",
                package="terrauto_ndt_scan_matcher",
                executable="ndt_scan_matcher",
                parameters=[
                    localization_component_params_file,
                ],
                remappings=[
                    ("points", "/voxel_grid_filter/points"),
                ],
                output="screen",
            ),
        ],
    )

    return LaunchDescription(
        [
            declare_localization_component_params_file_cmd,
            declare_use_composition_cmd,
            declare_use_multithread_cmd,
            set_container_executable_single,
            set_container_executable_multi,
            load_composition_nodes,
            load_nodes,
        ]
    )
