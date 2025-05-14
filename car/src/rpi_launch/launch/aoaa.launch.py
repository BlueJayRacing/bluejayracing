import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('rpi_launch'),  # <-- Replace with your actual package name
        'config',
        'inertial_sense_imx.yaml'
    )

    # Launch the Inertial Sense ROS 2 node with our YAML config
    inertial_sense_node = Node(
        package='inertial_sense_ros2',           # Or 'inertial_sense_ros' if that's your driver package name
        executable='inertial_sense_ros2_node',                 # Replace with the actual executable if it differs
        name='inertial_sense_driver_node',       # Node name override
        output='screen',
        parameters=[config_file]
    )


    car_pkg_path = get_package_share_directory('car')
    car_nodes_launch_path = os.path.join(car_pkg_path, 'launch', 'car_nodes.launch.py')
    car_nodes_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([car_nodes_launch_path])
    )

    udp_data_chunk_server = Node(
        package="udp_data_chunk_server",
        executable="udp_data_chunk_server",
        output="screen",
    )
    data_api_http_server = Node(
        package="data_api_http_server",
        executable="aggregation_server_node",
        output="screen",
    )

    return LaunchDescription([
        inertial_sense_node,
        car_nodes_launch,
        udp_data_chunk_server,
        data_api_http_server
    ])
