import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    motor_config = os.path.join(
        get_package_share_directory('rov'),
        'config',
        'motor_force_config_ENU.yaml'
        )
    
    thruster_config = os.path.join(
        get_package_share_directory('rov'),
        'config',
        'thruster.yaml'
        )
    node=Node(
        package = 'rov_thruster',
        name = 'thruster_manager',
        executable = 'thruster_manager',
        output = 'screen',
        parameters = [motor_config,
                    #   thruster_config,
                      {"wrench_sub_topic": "control_effort"},
                      {"cmd_sub_topic": "command"},
                      {"thrust_cmd_pub_topic": "thruster_command"},
                      ]
    )

    ld.add_action(node)
    return ld