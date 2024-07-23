from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package="teleop_twist_keyboard",
        #     executable="teleop_twist_keyboard",
        #     # input="screen",
        #     output="screen"
        # ),
        Node(
            package="brunhilde_teleop",
            executable="telestat",
            output="screen"
        ),
    ])