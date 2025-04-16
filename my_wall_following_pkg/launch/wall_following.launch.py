from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_wall_following_pkg',
            executable='estimator',
            name='estimator',
            output='screen'
        ),
        Node(
            package='my_wall_following_pkg',
            executable='controller',
            name='controller',
            output='screen'
        )
    ])
