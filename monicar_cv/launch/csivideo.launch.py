#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  cv_parameter = LaunchConfiguration(
    'cv_parameter',
    default=os.path.join(
      get_package_share_directory('monicar_cv'),
      'param/cvparam.yaml'
    )
  )

  return LaunchDescription([
    DeclareLaunchArgument('cv_parameter', default_value=cv_parameter),

    Node(
      package='monicar_cv', executable='csi_pub', name='csicam_node',
      output='screen', emulate_tty=True,
    ),
    Node(
        package='image_view',  executable='image_saver',  name='image_saver',
        remappings=[
            ('image', '/image_raw')
        ],
        arguments=['_save_all_image:=false', '_filename_format:=/path/to/save/image%04i.jpg']
    )
    ])
