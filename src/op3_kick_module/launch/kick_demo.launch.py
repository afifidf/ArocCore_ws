#!/usr/bin/env python3
# =============================================================================
# Launch file untuk Demo Kick Module dengan Soccer Demo
# =============================================================================
# 
# Launch file ini menjalankan kick module bersama dengan komponen lain
# yang diperlukan untuk demo tendangan.
#
# Penggunaan:
#   ros2 launch op3_kick_module kick_demo.launch.py
#
# =============================================================================

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    kick_module_share = get_package_share_directory('op3_kick_module')
    
    # Config file
    kick_config = os.path.join(kick_module_share, 'config', 'kick_config.yaml')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Kick Controller Node
    kick_controller_node = Node(
        package='op3_kick_module',
        executable='kick_module_node',
        name='kick_controller',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        arguments=[kick_config]
    )
    
    # Kick Command Publisher Node (untuk testing)
    # Uncomment untuk testing manual
    # kick_test_pub = Node(
    #     package='op3_kick_module',
    #     executable='kick_test_publisher',
    #     name='kick_test_publisher',
    #     output='screen'
    # )
    
    return LaunchDescription([
        use_sim_time_arg,
        kick_controller_node,
    ])
