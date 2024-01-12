from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription(
        [
            Node(
                package="muuv_sensors",
                name="imu_node",
                executable="imu_node",
                emulate_tty=True,
                parameters=[
                    {"pi_address": "192.168.8.157"},
                ],
                output="screen",
            ),
        ]
    )
    return ld
