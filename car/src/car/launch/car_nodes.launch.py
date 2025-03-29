import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    wsg_update_time_node = Node(
        package="car",
        executable="wsg_update_time_node",
        output="screen",
    )

    wsg_drive_data_node  = Node(
        package="car",
        executable="wsg_drive_data_node",
        output="screen",
    )

    broker_node = Node(
        package="car",
        executable="broker_node",
        output="screen",
    )

    writer_node = Node(
        package="car",
        executable="writer_node",
        output="screen",
    )

    transmit_prioritizer_node = Node(
        package="car",
        executable="transmit_prioritizer_node",
        output="screen",
    )

    xbee_node = Node(
        package="car",
        executable="xbee_node",
        output="screen",
    )

    return LaunchDescription(
        [
            wsg_update_time_node,
            wsg_drive_data_node,
            broker_node,
            writer_node,
            transmit_prioritizer_node,
            # xbee_node,
        ]
    )