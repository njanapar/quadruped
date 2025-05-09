# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. All rights reserved.

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.actions import RegisterEventHandler, SetEnvironmentVariable , AppendEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command,FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    robot_control_pkg_path = os.path.join(
        get_package_share_directory('spot_control'))
    
    spotrobot_control_path = os.path.join(
        get_package_share_directory('spotrobot_control'))
    


    def robot_state_publisher(context):
        performed_description_format = LaunchConfiguration('description_format').perform(context)
        # Get URDF or SDF via xacro
        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                PathJoinSubstitution([
                    FindPackageShare('spot_control'),
                    performed_description_format,
                    f'robot.xacro.{performed_description_format}'
                ]),
            ]
        )
        robot_description = {'robot_description': robot_description_content}
        node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        )
        return [node_robot_state_publisher]
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('spot_control'),
            'config',
            'robot_controller.yaml',
        ]
    )
    
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('spot_control'),
                     'meshes'))
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(robot_control_pkg_path, 'meshes'), ':' +
            str(Path(robot_control_pkg_path).parent.resolve())
            ]
        )
    

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'quad_robot', '-allow_renaming', 'true'],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
                   '/model/acs_robot/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                   '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'],
        output='screen'
    )

    manual_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(spotrobot_control_path, 'launch', 'robot_control.launch.py'),
        )
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller',
                   '--param-file',
                   robot_controllers,
                   ],
    )


    ld = LaunchDescription([
        bridge,
        gazebo_resource_path,
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 1 empty.sdf'])]),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[forward_position_controller_spawner],
            )
        ),
        gz_spawn_entity,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        DeclareLaunchArgument(
            'description_format',
            default_value='urdf',
            description='Robot description format to use, urdf or sdf'),
        manual_control,
    ])
    ld.add_action(OpaqueFunction(function=robot_state_publisher))
    ld.add_action(set_env_vars_resources)
    ld.add_action(gazebo_resource_path)
    return ld