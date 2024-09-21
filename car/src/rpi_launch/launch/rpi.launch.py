import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    car_description_pkg_path = get_package_share_directory('car_description')
    bts_control_launch_path = os.path.join(car_description_pkg_path, 'launch', 'bts_control.launch.py')
    bts_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bts_control_launch_path])
    )

    inertial_sense_ros_humble_pkg_path = get_package_share_directory('inertial_sense_ros_humble')
    node_with_param_launch_path = os.path.join(inertial_sense_ros_humble_pkg_path, 'launch', 'node_with_param.launch.py')
    node_with_param_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([node_with_param_launch_path])
    )

    car_pkg_path = get_package_share_directory('car')
    car_nodes_launch_path = os.path.join(car_pkg_path, 'launch', 'car_nodes.launch.py')
    car_nodes_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([car_nodes_launch_path])
    )

    return LaunchDescription(
        [
            bts_control_launch,
            node_with_param_launch,
            car_nodes_launch
        ]
    )