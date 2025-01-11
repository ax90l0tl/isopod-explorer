import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    motor_config =PathJoinSubstitution(
        [FindPackageShare('rov'),
        'config',
        'motor_force_config_ENU.yaml'
        ])
    
    thruster_config =PathJoinSubstitution(
        [FindPackageShare('rov'),
        'config',
        'thruster.yaml'
        ])
    
    

    node=Node(
        package = 'rov_thruster',
        name = 'thruster_manager',
        executable = 'thruster_manager',
        output = 'screen',
        parameters = [
                      {"wrench_sub_topic": "control_effort"},
                      {"cmd_sub_topic": "command"},
                      {"thrust_cmd_pub_topic": "thruster_command"},
                      thruster_config,
                      motor_config,
                      ],
    )

    ld.add_action(node)
    return ld