from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    lifecycle_state_machine = Node(
        package="athena_lifecycle_state_machine",
        executable="lifecycle_state_machine",
        name="lifecycle_state_machine",
        output="screen",
    )

    return LaunchDescription([lifecycle_state_machine])
