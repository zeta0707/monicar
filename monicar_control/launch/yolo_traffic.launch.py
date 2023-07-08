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
  traffic_parameter = LaunchConfiguration(
    'traffic_parameter',
    default=os.path.join(
      get_package_share_directory('monicar_control'),
      'param/traffic.yaml'
    )
  )

  return LaunchDescription([
    DeclareLaunchArgument('traffic_parameter', default_value=traffic_parameter),

    Node(
      package='monicar_control', executable='chase_traffic_yolo', name='traffic_node', 
	    output='screen', emulate_tty=True,
      parameters=[traffic_parameter],
    ),
    
  ])