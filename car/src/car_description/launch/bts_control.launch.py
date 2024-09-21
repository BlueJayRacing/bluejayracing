from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("car_description"), "urdf", "car.urdf.xacro"]),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    controllers_config = PathJoinSubstitution([FindPackageShare("baja_control"), "config", "control.yaml"])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_config],
        output="both"
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    ackermann_steering_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_steering_controller", "-c", "/controller_manager"],
    )

    twist_keyboard_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        output="both",
        remappings=[("/cmd_vel", "/ackermann_steering_controller/reference_unstamped")]
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            control_node,
            joint_state_broadcaster,
            ackermann_steering_controller,
            # twist_keyboard_node,
        ]
    )