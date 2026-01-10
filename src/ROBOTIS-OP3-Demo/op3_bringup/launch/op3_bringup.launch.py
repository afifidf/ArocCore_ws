# =============================================================================
# op3_bringup/launch/op3_bringup.launch.py
# =============================================================================
# Launch file UTAMA untuk menjalankan robot ROBOTIS OP3
# 
# Menjalankan:
# 1. op3_manager - Controller utama robot (motor control, sensor)
# 2. usb_cam     - Driver kamera USB untuk vision
#
# Cara menjalankan:
#   ros2 launch op3_bringup op3_bringup.launch.py
#
# Pastikan:
# - Robot terhubung via USB (/dev/ttyUSB0)
# - Kamera terhubung (/dev/video0)
# =============================================================================

# =============================================================================
# IMPORT LIBRARIES
# =============================================================================
import launch                                           # Library launch ROS2
from launch import LaunchDescription                    # Class utama launch description
from launch_ros.actions import Node                     # Untuk menjalankan node
from launch.actions import IncludeLaunchDescription     # Untuk include launch file lain
from launch.launch_description_sources import PythonLaunchDescriptionSource  # Source launch Python
from ament_index_python.packages import get_package_share_directory  # Cari path package


# =============================================================================
# GENERATE LAUNCH DESCRIPTION
# =============================================================================
def generate_launch_description():
  """
  Generate konfigurasi launch untuk bringup robot OP3
  
  Nodes/Launch yang dijalankan:
  1. op3_manager.launch.py - Manager robot (motor, sensor)
  2. usb_cam_node - Driver kamera USB
  
  Returns:
      LaunchDescription object
  """
  
  # ===========================================================================
  # INCLUDE OP3 MANAGER LAUNCH
  # ===========================================================================
  # Include launch file dari package op3_manager
  # Ini akan menjalankan node controller utama robot
  op3_manager_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [get_package_share_directory('op3_manager'), '/launch/op3_manager.launch.py']
    )
  )

  # ===========================================================================
  # USB CAMERA NODE
  # ===========================================================================
  # Node untuk membaca gambar dari kamera USB
  # Publish ke topic: /usb_cam_node/image_raw
  usb_cam_node = Node(
    package='usb_cam',                      # Package name
    executable='usb_cam_node_exe',          # Executable name
    name='usb_cam_node_exe',                # Node name saat running
    output='log',                           # Output ke log (bukan screen)
    parameters=[{
      # === DEVICE SETTINGS ===
      'video_device': '/dev/video0',        # Path device kamera (sesuaikan!)
      
      # === RESOLUSI & FPS ===
      'image_width': 1280,                  # Lebar gambar (pixel)
      'image_height': 720,                  # Tinggi gambar (pixel)
      'framerate': 30.0,                    # Frame per second
      
      # === FRAME ID ===
      'camera_frame_id': 'cam_link',        # TF frame ID untuk kamera
      'camera_name': 'camera',              # Nama kamera
      
      # === I/O SETTINGS ===
      'io_method': 'mmap',                  # Metode I/O (mmap lebih cepat)
      'pixel_format': 'mjpeg2rgb',          # Format pixel (MJPEG decode ke RGB)
      'av_device_format': 'YUV422P',        # Format device internal
    }],
    # Remap topic output ke nama yang diharapkan oleh node lain
    remappings=[('/image_raw', '/usb_cam_node/image_raw')]
  )

  # ===========================================================================
  # RETURN LAUNCH DESCRIPTION
  # ===========================================================================
  return LaunchDescription([
    op3_manager_launch,     # 1. Jalankan op3_manager dulu
    usb_cam_node            # 2. Kemudian jalankan kamera
  ])
