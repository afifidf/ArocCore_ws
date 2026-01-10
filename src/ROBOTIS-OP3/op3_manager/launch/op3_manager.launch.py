# =============================================================================
# ROBOTIS-OP3/op3_manager/launch/op3_manager.launch.py
# =============================================================================
# Launch file untuk OP3 Manager - Controller utama robot ROBOTIS OP3
#
# OP3 Manager bertanggung jawab untuk:
# - Komunikasi dengan motor Dynamixel via USB
# - Membaca sensor (IMU, button, dll) dari OpenCR
# - Menjalankan motion modules (walking, action, head control)
#
# Cara menjalankan:
#   ros2 launch op3_manager op3_manager.launch.py
#
# Pastikan:
# - Robot terhubung ke PC via USB (/dev/ttyUSB0)
# - User punya permission untuk akses serial port
# =============================================================================

# =============================================================================
# IMPORT LIBRARIES
# =============================================================================
import launch                                           # Library launch ROS2
from launch import LaunchDescription                    # Class utama launch description
from ament_index_python.packages import get_package_share_directory  # Cari path package
from launch_ros.actions import Node                     # Untuk menjalankan node
from launch.substitutions import LaunchConfiguration    # Untuk substitusi nilai


# =============================================================================
# GENERATE LAUNCH DESCRIPTION
# =============================================================================
def generate_launch_description():
    """
    Generate konfigurasi launch untuk OP3 Manager
    
    Node yang dijalankan:
    - op3_manager : Controller utama robot OP3
    
    Returns:
        LaunchDescription object
    """
    
    # =========================================================================
    # DEFAULT PARAMETER VALUES
    # =========================================================================
    
    # Mode simulasi Gazebo (True = simulasi, False = robot asli)
    gazebo_default = False
    
    # Nama robot di Gazebo (jika mode simulasi)
    gazebo_robot_name_default = 'robotis_op3'
    
    # Path ke file konfigurasi
    # offset.yaml      : Offset kalibrasi untuk setiap joint motor
    # OP3.robot        : Definisi robot (joint names, ID motor, dll)
    # dxl_init_OP3.yaml: Konfigurasi inisialisasi Dynamixel
    offset_file_path_default = get_package_share_directory('op3_manager') + '/config/offset.yaml'
    robot_file_path_default = get_package_share_directory('op3_manager') + '/config/OP3.robot'
    init_file_path_default = get_package_share_directory('op3_manager') + '/config/dxl_init_OP3.yaml'
    
    # Device serial port untuk komunikasi dengan robot
    # Biasanya /dev/ttyUSB0, bisa berubah tergantung urutan koneksi USB
    device_name_default = '/dev/ttyUSB0'

    # =========================================================================
    # RETURN LAUNCH DESCRIPTION
    # =========================================================================
    return LaunchDescription([
        # =====================================================================
        # OP3 MANAGER NODE
        # =====================================================================
        # Node utama yang mengontrol seluruh hardware robot
        Node(
            package='op3_manager',              # Nama package
            executable='op3_manager',           # Nama executable
            output='screen',                    # Output log ke terminal
            parameters=[{
                # === GENERAL SETTINGS ===
                'angle_unit': 30.0,             # Unit sudut untuk konversi
                
                # === GAZEBO SIMULATION ===
                'gazebo': gazebo_default,                       # Mode simulasi on/off
                'gazebo_robot_name': gazebo_robot_name_default, # Nama robot di Gazebo
                
                # === FILE PATHS ===
                'offset_file_path': offset_file_path_default,   # File offset kalibrasi
                'robot_file_path': robot_file_path_default,     # File definisi robot
                'init_file_path': init_file_path_default,       # File init Dynamixel
                
                # === HARDWARE ===
                'device_name': device_name_default              # Serial port USB
            }]
        )
        # Node localization (dinonaktifkan - uncomment jika diperlukan)
        # Node(
        #     package='op3_localization',
        #     executable='op3_localization',
        #     name='op3_localization',
        #     output='screen'
        # )
    ])
