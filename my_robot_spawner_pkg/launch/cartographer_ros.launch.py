from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_spawner_pkg')
    config_dir = os.path.join(pkg_dir, 'config', 'cartographer')

    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'warehouse.lua',
            ],
        )
    ])
