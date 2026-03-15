# main_task26.launch.py — Launch file untuk state machine AROC26
#
# Menjalankan node:
#   1. button_soccer_node (buttonHandler26) — handler tombol OpenCR
#
# Node ini sudah include TaskControl dari main_task26 di dalamnya.
# Jalankan ini SETELAH headControl26 sudah running.
#
# Cara menjalankan:
#   ros2 launch main_task26 main_task26.launch.py
#
# Optional:
#   ros2 launch main_task26 main_task26.launch.py log_level:=debug

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level: debug, info, warn, error'
    )
    log_level = LaunchConfiguration('log_level')

    # ── button_soccer_node ────────────────────────────────────
    # Bertugas:
    #   - Terima tombol OpenCR (/robotis/open_cr/button)
    #   - Teruskan ke TaskControl (main_task26)
    #   - TaskControl handle: standup, sit, approach bola
    button_node = Node(
        package='buttonHandler26',
        executable='button_soccer',
        name='button_soccer_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription([
        log_level_arg,
        button_node,
    ])
