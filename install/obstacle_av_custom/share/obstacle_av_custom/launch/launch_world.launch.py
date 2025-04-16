import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare a launch argument for the world file location
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='/home/suba/my_ros2_ws/src/obstacle_av_custom/world/mera_ghar', description='Path to the Gazebo world file'),

        # Include the gazebo_ros launch file to automatically start Gazebo with the world
        Node(
            package='gazebo_ros',
            executable='gzserver',
            name='gazebo_server',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-r', LaunchConfiguration('world')]  # Use the world file path here
        ),
        Node(
            package='gazebo_ros',
            executable='gzclient',
            name='gazebo_client',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])
