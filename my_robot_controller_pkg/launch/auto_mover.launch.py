from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_controller_pkg',
            executable='auto_mover',
            name='auto_mover',
            output='screen'
        )
    ])
