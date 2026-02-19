# loc: aroc26/src/headControl26/launch/headControl26.launch.py
#
# ============================================================
# Launch file untuk package headControl26
# ============================================================
# Menjalankan dua node secara bersamaan:
#
#   1. vision_node  — Subscriber kamera (/image_raw),
#                     inferensi YOLO OpenVINO, publish posisi
#                     bola ke /obj_detect
#
#   2. headcontrol_node — Subscriber /obj_detect,
#                         filter EKF posisi bola,
#                         PID head pan/tilt,
#                         publish ke /robotis/head_control/set_joint_states
#
# Cara menjalankan:
#   ros2 launch headControl26 headControl26.launch.py
#
# Optional argument:
#   ros2 launch headControl26 headControl26.launch.py log_level:=debug
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
    # Node 1: vision_node
    # Bertugas:
    #   - Subscribe /image_raw dari USB kamera
    #   - Jalankan inferensi YOLO OpenVINO
    #   - Publish koordinat pixel bola ke /obj_detect (String)
    #   - Tampilkan frame dengan anotasi (cv2.imshow)
    # ============================================================
    vision_node = Node(
        package='headControl26',
        executable='vision',
        name='vision_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )

    # ============================================================
    # Node 2: headcontrol_node
    # Bertugas:
    #   - Subscribe /obj_detect (posisi bola raw dari YOLO)
    #   - Update EKF dengan measurement baru
    #   - Hitung error pan/tilt dari posisi EKF (smooth)
    #   - Jalankan PID untuk menggerakkan head servo
    #   - Publish JointState ke /robotis/head_control/set_joint_states
    #   - Publish debug ke /vision/ball_measurement dan /vision/ball_ekf
    # ============================================================
    headcontrol_node = Node(
        package='headControl26',
        executable='headcontrol',
        name='headcontrol_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription([
        log_level_arg,
        vision_node,
        headcontrol_node,
    ])
