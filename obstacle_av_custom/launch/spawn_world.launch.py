import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the URDF and world files
    world_file = os.path.join(get_package_share_directory('obstacle_av_custom'), 'world', 'my_world')
    urdf_file = os.path.join(get_package_share_directory('obstacle_av_custom'), 'urdf', 'my_robot.urdf')

    # Create the launch description
    return LaunchDescription([
        # Set Gazebo environment variables
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(get_package_share_directory('obstacle_av_custom'), 'models')),

        # Spawn world
        Node(
            package='gazebo_ros',
            executable='gazebo',
            name='gazebo',
            output='screen',
            arguments=[world_file]
        ),
        
        # Spawn robot using urdf
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_my_robot',
            output='screen',
            arguments=['-entity', 'my_robot', '-file', urdf_file]
        ),
    ])

