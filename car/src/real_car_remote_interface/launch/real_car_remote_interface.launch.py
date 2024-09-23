import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import FindExecutable, LaunchConfiguration

def generate_launch_description():
    keyboard_cmd_vel = ExecuteProcess(
        cmd=[[
            FindExecutable(name='gnome-terminal'),
            ' --',
            ' ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p use_sim_time:=false'
        ]],
        shell=True
    )

    rviz = Node(
        package='rviz2', executable='rviz2',
        output='screen',
        arguments=['-d', str(os.path.join(get_package_share_directory('rviz_config'), 'config/config.rviz'))],
        parameters=[{
            "use_sim_time": False
        }]
    )

    camera_launch = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' launch',
            ' realsense2_camera rs_launch.py',
            ' pointcloud.enable:=true'
        ]],
        shell=True
    )

    cmd_vel_bridge_gz_to_ackermann = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"],
        remappings=[
            ('/cmd_vel', '/ackermann_steering_controller/reference_unstamped')
        ]
    )

    cmd_vel_bridge_nav2_to_gz = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"]
    )

    return LaunchDescription([
        keyboard_cmd_vel,
        rviz,
        camera_launch,
        cmd_vel_bridge_gz_to_ackermann,
        cmd_vel_bridge_nav2_to_gz
    ])