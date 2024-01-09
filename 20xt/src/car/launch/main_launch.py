
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='car', # Package Name
            executable='main', # Executable file
            output='screen', # Type of Output channel
            emulate_tty=True), # Log Message
    ])