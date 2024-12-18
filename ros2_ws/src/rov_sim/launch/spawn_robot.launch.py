import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
    package_name='rov_sim'

    name = LaunchConfiguration('name')
    name_launch_arg = DeclareLaunchArgument(
            'name',
            default_value='isopod',
            description='Robot Name'
    )
    
    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(get_package_share_directory('rov_sim'),'meshes')
    )
    
    
    robot_desc = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','demo.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    # Include the Gazebo launch file, provided by the gazebo_ros package
    bridge_params = os.path.join(
        get_package_share_directory('rov_sim'),
        'config',
        'ros_gz_bridge.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', name,
                                   '-x', '0.0', 
                                   '-y', '0.0',
                                   '-z', '0.1',
                                   ],
                        output='screen')

    ld = LaunchDescription()

    # Launch them all!
    ld.add_action(name_launch_arg)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    # ld.add_action(start_gazebo_ros_image_bridge_cmd)
    ld.add_action(set_env_vars_resources)
    ld.add_action(robot_desc)
    ld.add_action(spawn_entity)
    return ld