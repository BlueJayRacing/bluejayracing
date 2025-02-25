import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    mqtt_client_echo = Node(
        package="test",
        executable="mqtt_client_echo",
        output="screen",
    )

    return LaunchDescription(
        [
            mqtt_client_echo,
        ]
    )