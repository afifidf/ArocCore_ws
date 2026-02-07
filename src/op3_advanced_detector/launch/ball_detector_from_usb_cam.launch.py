# =============================================================================
# ball_detector_from_usb_cam.launch.py
# =============================================================================
# Part of launch package
# =============================================================================

# op3_advanced_detector/launch/ball_detector_from_usb_cam.launch.py

"""
Launch file for OP3 Advanced Ball Detector with USB Camera
This replaces the original op3_ball_detector launch file with YOLO-based detection
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# =============================================================================
# VENV CONFIGURATION
# =============================================================================
# Path ke virtual environment yang berisi ultralytics
# Prioritas: VIRTUAL_ENV environment variable > default path
VENV_PATH = os.environ.get('VIRTUAL_ENV', os.path.expanduser('~/.venvs/yolo'))
VENV_SITE_PACKAGES = os.path.join(VENV_PATH, 'lib', 'python3.12', 'site-packages')


def generate_launch_description():
    ld = LaunchDescription()

    # Package path
    pkg_share = FindPackageShare('op3_advanced_detector')
    camera_param_path = PathJoinSubstitution([pkg_share, 'config', 'camera_param.yaml'])
    detector_config_path = PathJoinSubstitution([pkg_share, 'config', 'detector_config.yaml'])

    # USB Camera Node (sama seperti original)
    usb_cam_node = Node(
        package='usb_cam',
        namespace='usb_cam_node',
        executable='usb_cam_node_exe',
        output='screen',
        parameters=[camera_param_path],
    )

    # Argumen untuk config detector
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=detector_config_path,
        description='Ball detector configuration file path (YAML)',
    )

    # Advanced Ball Detector Node (YOLO)
    ball_detector_node = Node(
        package='op3_advanced_detector',
        executable='advanced_detector',  # kita ganti dari op3_advanced_detector
        name='op3_advanced_detector',
        output='screen',
        emulate_tty=True,  # biar warna log tetap muncul
        parameters=[
            LaunchConfiguration('config_file'),
            {
                # Pastikan ini sama dengan topic image dari usb_cam
                'camera_topic': '/usb_cam_node/image_raw',
            },
        ],
        # Tambahkan PYTHONPATH dari venv agar bisa import ultralytics
        additional_env={
            'PYTHONPATH': VENV_SITE_PACKAGES + ':' + os.environ.get('PYTHONPATH', '')
        },
    )

    ld.add_action(config_file_arg)
    ld.add_action(usb_cam_node)
    ld.add_action(ball_detector_node)

    return ld
