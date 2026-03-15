# aroc26.launch.py — Launch file utama AROC26
# Menjalankan semua package sekaligus:
#   1. vision_node       (headControl26) — deteksi bola & gawang via YOLO
#   2. headcontrol_node  (headControl26) — head tracking PID + EKF
#   3. button_soccer     (buttonHandler26) — handler tombol OpenCR
#   4. kicknrush_node    (KickNRush) — state machine orbit + crab + tendang
#
# Cara menjalankan:
#   ros2 launch aroc26_bringup aroc26.launch.py
#
# Optional:
#   ros2 launch aroc26_bringup aroc26.launch.py log_level:=debug

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    # Argument log level — berlaku untuk semua node
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level: debug, info, warn, error'
    )

    # Argument kalibrasi
    calibrate_arg = DeclareLaunchArgument(
        'calibrate',
        default_value='true',
        description='Jalankan GUI kalibrasi HSV bola dan gawang'
    )

    log_level = LaunchConfiguration('log_level')
    calibrate = LaunchConfiguration('calibrate')

    show_gui_arg = DeclareLaunchArgument(
        'show_gui',
        default_value='true',
        description='Tampilkan window OpenCV untuk vision_cv'
    )
    show_gui = LaunchConfiguration('show_gui')

    # ── Node 1: vision_node ───────────────────────────────────────
    # Subscribe /image_raw → inferensi YOLO → publish:
    #   /obj_detect           (posisi bola)
    #   /obj_detect_ball_bbox (bbox bola)
    #   /obj_detect_goal      (posisi gawang)
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

    # ── Node 2: headcontrol_node ──────────────────────────────────
    # Subscribe /obj_detect → EKF + PID → gerak kepala robot
    headcontrol_node = Node(
        package='headControl26',
        executable='headcontrol',
        name='headcontrol_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )

    # ── Node 3: button_soccer_node ────────────────────────────────
    # Subscribe /robotis/open_cr/button → kontrol manual via tombol
    # ⚠️ Catatan: saat KickNRush aktif, tombol sudah di-handle
    #    oleh task_control. buttonHandler26 tetap dijalankan
    #    sebagai fallback manual kontrol.
    button_node = Node(
        package='buttonHandler26',
        executable='button_soccer',
        name='button_soccer_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )

    # ── Node 4: kicknrush_node ────────────────────────────────────
    # State machine utama: SEARCH → APPROACH → ORBIT → ADJUST → KICK
    kicknrush_node = Node(
        package='KickNRush',
        executable='kicknrush',
        name='kicknrush_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )

    # ── Node 5: calibrate_node ────────────────────────────────────
    # Tool GUI untuk kalibrasi HSV (aktif jika calibrate:=true)
    calibrate_node = Node(
        package='headControl26',
        executable='calibrate',
        name='calibrate_node',
        output='screen',
        condition=IfCondition(calibrate)
    )

    return LaunchDescription([
        log_level_arg,
        calibrate_arg,
        show_gui_arg,
        vision_node,
        headcontrol_node,
        button_node,
        kicknrush_node,
        calibrate_node,
    ])
