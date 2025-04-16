#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    launch_file_dir = os.path.join(get_package_share_directory('obstacle_avoidance'), 'launch')

    turtlebot3_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        )
    )
    obstacle_avoider_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "obstacle_avoider.launch.py")
        )
    )

    ld = LaunchDescription()
    ld.add_action(turtlebot3_world_cmd)
    ld.add_action(obstacle_avoider_cmd)
    return ld
