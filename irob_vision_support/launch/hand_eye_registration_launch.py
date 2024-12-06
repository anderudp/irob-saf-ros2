from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='irob_vision_support',
            executable='cylmarker_detector',
            name='cylmarker_detector',
            parameters=[
                {"config_folder_path": "/root/ros2_ws/src/irob-saf-ros2/irob_vision_support/config"},  # TODO: Hardcoded for Docker container
                {"data_path": "/root/ros2_ws/src/irob-saf-ros2/irob_vision_support/data"}
            ]
        )
    ])