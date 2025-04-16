#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    obstacle_avoider_node = Node(
        package="obstacle_avoidance",
        executable="obstacle_avoider",
        name="obstacle_avoider"
    )

    return LaunchDescription([obstacle_avoider_node])
