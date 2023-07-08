#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
  yolo_track_parameter = LaunchConfiguration(
    'yolo_track_parameter',
    default=os.path.join(
      get_package_share_directory('monicar_control'),
      'param/yolo.yaml'
    )
  )

  return LaunchDescription([
    DeclareLaunchArgument(
      'yolo_track_parameter',
      default_value=yolo_track_parameter
    ),

    Node(
      package='monicar_control',  executable='chase_object_yolo',  name='chase_object_node',
      output='screen',  emulate_tty=True,
      parameters=[yolo_track_parameter],
      namespace='',
    )
  ])