import os
from launch import LaunchDescription
from launch_ros.actions import Node
 
 
def generate_launch_description():
 
  return LaunchDescription([
    Node(package='my_robot_controller_pkg', executable='robot_controller',
      output='screen'),
    Node(package='my_robot_controller_pkg', executable='robot_estimator',
      output='screen'),
  ])
