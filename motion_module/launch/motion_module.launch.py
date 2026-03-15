# motion_module.launch.py — Launch file untuk motion_module
#
# motion_module adalah library (bukan executable node),
# sehingga tidak ada node yang dijalankan di sini.
#
# Launch file ini disediakan sebagai REFERENSI cara pakai
# dan untuk include di launch file lain jika diperlukan.
#
# Cara include dari launch lain:
#   from launch.actions import IncludeLaunchDescription
#   from launch.launch_description_sources import PythonLaunchDescriptionSource
#   from ament_index_python.packages import get_package_share_directory
#   import os
#
#   motion_launch = IncludeLaunchDescription(
#       PythonLaunchDescriptionSource(
#           os.path.join(
#               get_package_share_directory('motion_module'),
#               'launch', 'motion_module.launch.py'
#           )
#       )
#   )
#
# Isi motion_module:
#   - motion_approach.py → MotionApproach (mendekati bola)
#   - orbit.py           → orbitalMotion  (orbit mengelilingi bola)

from launch import LaunchDescription


def generate_launch_description():
    # motion_module tidak punya executable node sendiri
    # Semua class-nya diimport langsung oleh node lain (main_task26, KickNRush)
    return LaunchDescription([])
