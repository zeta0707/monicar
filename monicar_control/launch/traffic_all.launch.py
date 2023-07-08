#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='monicar_cv', executable='csi_pub', name='camera_node', 
    ),
    
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        FindPackageShare("monicar_control"), '/launch', '/yolo_traffic.launch.py'])
    ), 

  ])

