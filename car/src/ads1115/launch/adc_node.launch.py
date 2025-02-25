from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    adc_node = Node(
        package="ads1115",
        executable="adc_node",
        output="screen",
    )

    return LaunchDescription([adc_node])