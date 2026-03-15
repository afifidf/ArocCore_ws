from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='buttonHandler26',
            executable='button_soccer',
            name='button_soccer_node',
            output='screen'
        )
    ])