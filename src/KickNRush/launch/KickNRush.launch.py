# loc: aroc26/src/KickNRush/launch/KickNRush.launch.py
#
# ============================================================
# Launch file untuk package KickNRush
# ============================================================
# Menjalankan node-node yang dibutuhkan untuk:
#   orbit ke bola → sejajar gawang → tendang
#
# Node yang dijalankan:
#   1. kicknrush_node  — Task control utama (orbit, crab, goal alignment, kick)
#
# Node yang TIDAK dijalankan di sini (sudah ada di headControl26.launch.py):
#   - vision_node     → sudah publish /obj_detect dan /obj_detect_goal
#   - headcontrol_node → sudah handle head tracking
#
# Cara menjalankan:
#   # Jalankan headControl26 dulu (vision + head tracking):
#   ros2 launch headControl26 headControl26.launch.py
#
#   # Lalu jalankan KickNRush di terminal lain:
#   ros2 launch KickNRush KickNRush.launch.py
#
# Optional argument:
#   ros2 launch KickNRush KickNRush.launch.py log_level:=debug
# ============================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ============================================================
    # Argument: log level (default info, bisa diubah ke debug)
    # ============================================================
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level untuk semua node (debug, info, warn, error)'
    )

    log_level = LaunchConfiguration('log_level')

    # ============================================================
    # Node: kicknrush_node
    # Bertugas:
    #   - Subscribe /obj_detect       → posisi bola dari vision
    #   - Subscribe /obj_detect_goal  → posisi gawang dari vision
    #   - Subscribe /robotis/open_cr/imu → data IMU untuk yaw
    #   - Subscribe /robotis/open_cr/button → tombol OpenCR
    #   - Jalankan state machine:
    #       SEARCH → APPROACH → ORBIT → ALIGN → KICK
    #   - Publish /robotis/walking/set_params → parameter jalan
    #   - Publish /robotis/walking/command    → start/stop walking
    #   - Publish /robotis/action/page_num    → nomor action tendang
    #   - Publish /goal_alignment/status      → status sejajar gawang
    #   - Publish /goal_alignment/debug       → info debug alignment
    # ============================================================
    kicknrush_node = Node(
        package='KickNRush',
        executable='kicknrush',
        name='kicknrush_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription([
        log_level_arg,
        kicknrush_node,
    ])
