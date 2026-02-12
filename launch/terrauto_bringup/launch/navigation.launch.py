#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the file directory
    bringup_share = get_package_share_directory("terrauto_bringup")

    map_path = os.path.join(bringup_share, "maps", "rtf.pcd")

    map_component_launch_path = os.path.join(
        bringup_share, "launch", "components", "map_component.launch.py"
    )
    sensing_component_launch_path = os.path.join(
        bringup_share, "launch", "components", "sensing_component.launch.py"
    )
    localization_component_launch_path = os.path.join(
        bringup_share, "launch", "components", "localization_component.launch.py"
    )

    map_component_params_path = os.path.join(bringup_share, "params", "map_component_params.yaml")
    sensing_component_params_path = os.path.join(
        bringup_share, "params", "sensing_component_params.yaml"
    )
    localization_component_params_path = os.path.join(
        bringup_share, "params", "localization_component_params.yaml"
    )

    # LaunchConfigurations
    map_file = LaunchConfiguration("map_file")
    map_component_params_file = LaunchConfiguration("map_component_params_file")
    sensing_component_params_file = LaunchConfiguration("sensing_component_params_file")
    localization_component_params_file = LaunchConfiguration("localization_component_params_file")
    use_composition = LaunchConfiguration("use_composition")
    use_multithread = LaunchConfiguration("use_multithread")
    use_intra_process = LaunchConfiguration("use_intra_process")

    # Declare args
    declare_map_file_cmd = DeclareLaunchArgument(
        "map_file",
        default_value=map_path,
        description="Full path to map file (PCD / PLY)",
    )
    declare_map_component_params_file_cmd = DeclareLaunchArgument(
        "map_component_params_file",
        default_value=map_component_params_path,
        description="Full path to the ROS 2 parameters file for map component nodes",
    )
    declare_sensing_component_params_file_cmd = DeclareLaunchArgument(
        "sensing_component_params_file",
        default_value=sensing_component_params_path,
        description="Full path to the ROS 2 parameters file for sensing component nodes",
    )
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
    declare_use_intra_process_cmd = DeclareLaunchArgument(
        "use_intra_process",
        default_value="True",
        description="Use intre-process communication for composable nodes",
    )

    # Load nodes
    load_nodes = GroupAction(
        [
            Node(
                name="base_to_lidar",
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "0.0",
                    "0.0",
                    "0.0",
                    "0.0",
                    "0.0",
                    "0.0",
                    "1.0",
                    "base_link",
                    "livox_frame",
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(map_component_launch_path),
                launch_arguments={
                    "map_file": map_file,
                    "map_component_params_file": map_component_params_file,
                    "use_composition": use_composition,
                    "use_multithread": use_multithread,
                    "use_intra_process": use_intra_process,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sensing_component_launch_path),
                launch_arguments={
                    "sensing_component_params_file": sensing_component_params_file,
                    "use_composition": use_composition,
                    "use_multithread": use_multithread,
                    "use_intra_process": use_intra_process,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(localization_component_launch_path),
                launch_arguments={
                    "localization_component_params_file": localization_component_params_file,
                    "use_composition": use_composition,
                    "use_multithread": use_multithread,
                    "use_intra_process": use_intra_process,
                }.items(),
            ),
        ]
    )

    return LaunchDescription(
        [
            declare_map_file_cmd,
            declare_map_component_params_file_cmd,
            declare_sensing_component_params_file_cmd,
            declare_localization_component_params_file_cmd,
            declare_use_composition_cmd,
            declare_use_multithread_cmd,
            declare_use_intra_process_cmd,
            load_nodes,
        ]
    )
