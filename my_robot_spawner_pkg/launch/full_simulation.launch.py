import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_spawner_pkg')
    world = os.path.join(pkg_dir, 'worlds', 'warehouse.world')
    config_dir = os.path.join(pkg_dir, 'config', 'cartographer')
    rviz_config = os.path.join(config_dir, 'carto.rviz')

    # RViz toggle argument
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    # Gazebo Server
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', world, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Gazebo Client
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Spawn Robot
    spawn_robot = Node(
        package='my_robot_spawner_pkg',
        executable='spawn_demo',
        arguments=['WarehouseBot', 'demo', '-1.5', '-4.0', '0.0'],
        output='screen'
    )

    # Cartographer Node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'warehouse.lua'
        ]
    )

    # RViz2 Node (conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        gazebo_server,
        gazebo_client,
        spawn_robot,
        cartographer_node,
        rviz_node
    ])

