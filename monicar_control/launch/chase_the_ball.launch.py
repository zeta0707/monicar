#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
  chaseball_parameter = LaunchConfiguration(
    'chaseball_parameter',
    default=os.path.join(
      get_package_share_directory('monicar_control'),
      'param/motor.4chase.yaml'
    )
  )

  return LaunchDescription([
    DeclareLaunchArgument('chaseball_parameter', default_value=chaseball_parameter),

    Node(
      package='monicar_control', executable='chase_the_ball', name='chase_ball_node', 
	    output='screen', emulate_tty=True,
      parameters=[chaseball_parameter],
    ),
    
  ])