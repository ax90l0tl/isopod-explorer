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

    default_world = os.path.join(
        get_package_share_directory('rov_sim'),
        'worlds',
        'underwater_basic.world'
    )

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
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
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )
    
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )


    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'rov',
                                   '-x', '0.0', 
                                   '-y', '0.0',
                                   '-z', '0.1',
                                   ],
                        output='screen')



    # Launch them all!
    return LaunchDescription([
        # set_env_vars_resources,
        world_arg,
        robot_desc,
        gzserver_cmd,
        gzclient_cmd,
        spawn_entity,
    ])
