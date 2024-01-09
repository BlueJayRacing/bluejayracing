from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensors', # Package Name
            executable='publisher', # Executable file
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='sensors',
            executable='subscriber',
            output='screen',
            emulate_tty=True,
        ),
    ])