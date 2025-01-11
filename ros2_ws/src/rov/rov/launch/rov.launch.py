import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable, GroupAction
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    namespace = LaunchConfiguration('name')
    namespace_arg = DeclareLaunchArgument(
        'name',
        default_value='isopod',
        description='robot namespace'
    )
    thruster_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rov_thruster'), 'launch'),
            '/thruster_manager.launch.py'
        ]), launch_arguments={'use_sim_time': 'true'}.items()
    )


    # gives all nodes the namespace isopod (recursive)
    sim_w_namespace = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            thruster_manager
        ]
    )

    ld = LaunchDescription()

    # Launch them all!
    ld.add_action(thruster_manager)
    ld.add_action(namespace_arg)
    # ld.add_action(sim_w_namespace)
    return ld