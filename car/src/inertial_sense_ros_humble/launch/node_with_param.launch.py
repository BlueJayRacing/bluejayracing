import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Parameters
    params_file = LaunchConfiguration('params_file', default='params.yaml')
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('inertial_sense_ros_humble'),
            'config',
            'params.yaml'
        ]),
        description='Path to the parameter file'
    )

    # Node configuration
    inertial_sense_node = Node(
        package='inertial_sense_ros_humble',
        executable='inertial_sense_node',
        name='inertial_sense_node',
        output='screen',
        parameters=[params_file]
    )

    # Launch description
    return LaunchDescription([
        params_file_arg,
        inertial_sense_node
    ])
