import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare the world file location
        DeclareLaunchArgument('world', default_value='/home/suba/my_ros2_ws/src/obstacle_av_custom/world/mera_ghar', description='Path to the Gazebo world file'),

        # Start the Gazebo server (gzserver)
        Node(
            package='gazebo_ros',
            executable='gzserver',
            name='gazebo_server',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[LaunchConfiguration('world')]  # Pass world path as argument
        ),
        
        # Start the Gazebo client (gzclient)
        Node(
            package='gazebo_ros',
            executable='gzclient',
            name='gazebo_client',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])

