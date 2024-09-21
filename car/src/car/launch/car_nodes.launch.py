import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ads1115_pkg_path = get_package_share_directory('ads1115')
    adc_node_launch_path = os.path.join(ads1115_pkg_path, 'launch', 'adc_node.launch.py')

    adc_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([adc_node_launch_path])
    )

    mqtt_client_node = Node(
        package="car",
        executable="mqtt_client_node",
        output="screen",
    )

    mqtt_client_publish_node = Node(
        package="car",
        executable="mqtt_client_publish_node",
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
            # adc_node_launch,
            mqtt_client_node,
            mqtt_client_publish_node,
            broker_node,
            writer_node,
            transmit_prioritizer_node,
            # xbee_node,
        ]
    )