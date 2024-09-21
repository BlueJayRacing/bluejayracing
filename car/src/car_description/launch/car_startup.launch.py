import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import FindExecutable, LaunchConfiguration

def generate_launch_description():
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_camera_turned_on = DeclareLaunchArgument(
        'camera_turned_on',
        default_value='false',
        description='Turn on rgbd camera if true. Expect severe performance drop',
    )
    camera_turned_on = LaunchConfiguration('camera_turned_on')

    publish_robot_state = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' run',
            ' robot_state_publisher robot_state_publisher',
            ' --ros-args -p',
            ' robot_description:=',
            '\"$(xacro ',
            str(os.path.join(get_package_share_directory('car_description'), 'urdf/car.urdf.xacro')),
            ' camera_on:=',
            camera_turned_on,
            ')\"',
            ' -p',
            ' use_sim_time:=',
            use_sim_time
        ]],
        shell=True
    )

    return LaunchDescription([
        publish_robot_state,
        declare_use_sim_time_cmd,
        declare_camera_turned_on
    ])