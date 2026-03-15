# loc: aroc26/src/headControl26/launch/headControl26.launch.py
#
# ============================================================
# Launch file untuk package headControl26
# ============================================================
# Menjalankan tiga node secara bersamaan:
#
#   1. usb_cam_node     — Driver kamera USB, publish /image_raw
#
#   2. vision_node      — Subscribe /image_raw,
#                         inferensi YOLO OpenVINO,
#                         publish /obj_detect, /obj_detect_ball_bbox,
#                         /obj_detect_goal
#
#   3. headcontrol_node — Subscribe /obj_detect,
#                         filter EKF + PID head pan/tilt,
#                         publish /robotis/head_control/set_joint_states
#
# Cara menjalankan:
#   ros2 launch headControl26 headControl26.launch.py
#
# Optional argument:
#   ros2 launch headControl26 headControl26.launch.py log_level:=debug
#   ros2 launch headControl26 headControl26.launch.py device:=/dev/video2
# ============================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    # ── Argument: log level ───────────────────────────────────
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level untuk semua node (debug, info, warn, error)'
    )

    # ── Argument: device kamera ───────────────────────────────
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/video0',
        description='Device kamera USB (default: /dev/video0)'
    )

    # ── Argument: kalibrasi ───────────────────────────────────
    calibrate_arg = DeclareLaunchArgument(
        'calibrate',
        default_value='true',
        description='Jalankan GUI kalibrasi HSV bola dan gawang'
    )

    log_level = LaunchConfiguration('log_level')
    device    = LaunchConfiguration('device')
    calibrate = LaunchConfiguration('calibrate')

    show_gui_arg = DeclareLaunchArgument(
        'show_gui',
        default_value='true',
        description='Tampilkan window OpenCV untuk vision_cv'
    )
    show_gui = LaunchConfiguration('show_gui')

    # ── Node 1: usb_cam_node ──────────────────────────────────
    # Driver kamera USB dari package usb_cam.
    # Publish /image_raw yang di-subscribe oleh vision_node.
    # Parameter:
    #   video_device → path device kamera (default /dev/video0)
    #   image_width  → lebar frame = 640 (sama dengan FRAME_W di vision.py)
    #   image_height → tinggi frame = 480 (sama dengan FRAME_H di vision.py)
    #   framerate    → 30 fps
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node',
        output='screen',
        parameters=[{
            'video_device': device,
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0,
            'pixel_format': 'yuyv',
            'camera_name': 'usb_cam',
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # ── Node 2: vision_node ───────────────────────────────────
    # Subscribe /image_raw → YOLO OpenVINO → publish deteksi bola & gawang.
    # Publish:
    #   /obj_detect           → "cx,cy" posisi bola
    #   /obj_detect_ball_bbox → "cx,cy,w,h" bbox bola (untuk estimasi jarak)
    #   /obj_detect_goal      → "cx,cy,w,h" bbox gawang
    vision_node = Node(
        package='headControl26',
        executable='vision_cv',
        name='vision_node',
        output='screen',
        parameters=[{
            'show_gui': show_gui,
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # ── Node 3: headcontrol_node ──────────────────────────────
    # Subscribe /obj_detect → EKF smooth → PID → gerak kepala.
    # Subscribe /head/state → "scan" aktifkan | "off" matikan.
    # Publish:
    #   /robotis/head_control/set_joint_states → posisi servo kepala
    #   /vision/ball_measurement               → posisi bola raw (debug)
    #   /vision/ball_ekf                       → posisi bola EKF (debug)
    headcontrol_node = Node(
        package='headControl26',
        executable='headcontrol',
        name='headcontrol_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )

    # ── Node 4: calibrate_node ────────────────────────────────
    # Subscribe /image_raw → tool GUI untuk kalibrasi HSV
    # Aktif jika argumen calibrate:=true
    calibrate_node = Node(
        package='headControl26',
        executable='calibrate',
        name='calibrate_node',
        output='screen',
        condition=IfCondition(calibrate)
    )

    return LaunchDescription([
        log_level_arg,
        device_arg,
        calibrate_arg,
        show_gui_arg,
        usb_cam_node,
        vision_node,
        headcontrol_node,
        calibrate_node,
    ])
