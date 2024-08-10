import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='prius_sdc_pkg',
            executable='recorder_node',
            name='video_recorder',
            output='screen'
        ),

        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='Car_drive',
            output='screen'
        ),
        
    ])