# =============================================================================
# op3_advanced_detector/launch/advanced_detector.launch.py
# =============================================================================
# Launch file untuk menjalankan node deteksi bola berbasis YOLO
# 
# Cara menjalankan:
#   ros2 launch op3_advanced_detector advanced_detector.launch.py
#   ros2 launch op3_advanced_detector advanced_detector.launch.py config_file:=/path/to/config.yaml
#
# Author: ROBOTIS (modified)
# =============================================================================

#!/usr/bin/env python3

# =============================================================================
# IMPORT LIBRARIES
# =============================================================================
import os
from launch import LaunchDescription                    # Class utama untuk launch description
from launch.actions import DeclareLaunchArgument        # Untuk deklarasi argument launch
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution  # Substitusi nilai
from launch_ros.actions import Node                     # Untuk menjalankan node ROS2
from launch_ros.substitutions import FindPackageShare   # Untuk mencari path package


# =============================================================================
# GENERATE LAUNCH DESCRIPTION
# =============================================================================
def generate_launch_description():
    """
    Generate konfigurasi launch untuk ball detector
    
    Node yang dijalankan:
    - op3_advanced_detector : Node deteksi bola menggunakan YOLO
    
    Arguments:
    - config_file : Path ke file konfigurasi YAML (opsional)
    
    Returns:
        LaunchDescription object
    """
    
    # Dapatkan path ke package share directory
    # Share directory berisi file config, launch, dll yang di-install
    pkg_share = FindPackageShare('op3_advanced_detector')
    
    # Path default ke file konfigurasi
    # File ini berisi parameter seperti: model YOLO, threshold, topic kamera, dll
    default_config_path = PathJoinSubstitution([
        pkg_share, 'config', 'detector_config.yaml'
    ])
    
    return LaunchDescription([
        # ==================================================================
        # LAUNCH ARGUMENTS
        # ==================================================================
        # Argument untuk path file konfigurasi
        # Bisa di-override saat launch: config_file:=/custom/path.yaml
        DeclareLaunchArgument(
            'config_file',                          # Nama argument
            default_value=default_config_path,      # Nilai default
            description='Ball detector configuration file path (YAML)'
        ),
        
        # ==================================================================
        # LAUNCH ARGUMENTS - VENV
        # ==================================================================
        # Argument untuk path ke virtual environment Python
        # Jika VIRTUAL_ENV sudah di-set di environment, gunakan itu
        # Jika tidak, gunakan path default ke venv 'yolo'
        DeclareLaunchArgument(
            'venv_path',
            default_value=os.environ.get('VIRTUAL_ENV', os.path.expanduser('~/.venvs/yolo')),
            description='Path to Python virtual environment containing ultralytics'
        ),
        
        # ==================================================================
        # NODES
        # ==================================================================
        # Node ball detector berbasis YOLO
        Node(
            package='op3_advanced_detector',        # Nama package
            executable='advanced_detector',         # Nama executable (dari setup.py)
            name='op3_advanced_detector',           # Nama node saat running
            parameters=[LaunchConfiguration('config_file')],  # Load parameter dari config
            output='screen',                        # Output log ke terminal
            emulate_tty=True,                       # Support warna di log output
            # Tambahkan PYTHONPATH dari venv agar bisa import ultralytics
            additional_env={
                'PYTHONPATH': os.path.join(
                    os.environ.get('VIRTUAL_ENV', os.path.expanduser('~/.venvs/yolo')),
                    'lib', 'python3.12', 'site-packages'
                ) + ':' + os.environ.get('PYTHONPATH', '')
            },
        )
    ])
