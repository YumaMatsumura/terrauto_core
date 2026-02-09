#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("terrauto_bringup")

    rviz_light_path = os.path.join(bringup_share, "rviz", "light.rviz")
    rviz_dark_path = os.path.join(bringup_share, "rviz", "dark.rviz")
    qss_light_path = os.path.join(bringup_share, "rviz", "light.qss")
    qss_dark_path = os.path.join(bringup_share, "rviz", "dark.qss")

    # LaunchConfigurations
    theme = LaunchConfiguration("theme")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    qss_file = LaunchConfiguration("qss_file")

    # Declare args
    declare_theme_cmd = DeclareLaunchArgument(
        "theme",
        default_value="light",
        description="UI theme for RViz2: 'light' or 'dark'",
        choices=["light", "dark"],
    )
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=rviz_light_path,
        description="Full path to the rviz config file (auto-set by theme by default)",
    )
    declare_qss_file_cmd = DeclareLaunchArgument(
        "qss_file",
        default_value=qss_light_path,
        description="Full path to the QSS file (auto-set by theme by default)",
    )

    # Apply theme selection (override launch configurations)
    set_light_theme_cmd = SetLaunchConfiguration(
        "rviz_config_file",
        rviz_light_path,
        condition=IfCondition(PythonExpression(["'", theme, "' == 'light'"])),
    )
    set_light_qss_cmd = SetLaunchConfiguration(
        "qss_file",
        qss_light_path,
        condition=IfCondition(PythonExpression(["'", theme, "' == 'light'"])),
    )

    set_dark_theme_cmd = SetLaunchConfiguration(
        "rviz_config_file",
        rviz_dark_path,
        condition=IfCondition(PythonExpression(["'", theme, "' == 'dark'"])),
    )
    set_dark_qss_cmd = SetLaunchConfiguration(
        "qss_file",
        qss_dark_path,
        condition=IfCondition(PythonExpression(["'", theme, "' == 'dark'"])),
    )

    # Load nodes
    rviz_node = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file, "--stylesheet", qss_file],
        output="screen",
    )

    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz_node,
            on_exit=EmitEvent(event=Shutdown(reason="rviz exited")),
        ),
    )

    return LaunchDescription(
        [
            declare_theme_cmd,
            declare_rviz_config_file_cmd,
            declare_qss_file_cmd,
            set_light_theme_cmd,
            set_light_qss_cmd,
            set_dark_theme_cmd,
            set_dark_qss_cmd,
            rviz_node,
            exit_event_handler,
        ]
    )
