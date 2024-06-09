#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='monicar_cv', executable='csi_pub', name='csicam_node',
      output='screen', emulate_tty=True,
    ),
  ])
