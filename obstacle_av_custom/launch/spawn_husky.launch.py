import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    husky_urdf_file = os.path.join(get_package_share_directory('obstacle_av_custom'), 'urdf', 'husky.urdf.xacro')

    return LaunchDescription([
        DeclareLaunchArgument('robot_description', default_value=husky_urdf_file, description='Path to the URDF file'),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': LaunchConfiguration('robot_description')}]
        ),
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_husky',
            output='screen',
            arguments=['-entity', 'husky', '-file', husky_urdf_file]
        ),
    ])
