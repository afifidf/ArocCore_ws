#!/usr/bin/env python3
# =============================================================================
# Launch file untuk OP3 Kick Module
# =============================================================================
# 
# Penggunaan:
#   ros2 launch op3_kick_module kick_module.launch.py
#   ros2 launch op3_kick_module kick_module.launch.py config_file:=/path/to/config.yaml
#
# =============================================================================

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Dapatkan path ke package
    pkg_share = get_package_share_directory('op3_kick_module')
    
    # Default config file path
    default_config = os.path.join(pkg_share, 'config', 'kick_config.yaml')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to kick module configuration file'
    )
    
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
        arguments=[LaunchConfiguration('config_file')],
        remappings=[
            # Remap jika diperlukan
            # ('/robotis/open_cr/imu', '/imu'),
            # ('/robotis/present_joint_states', '/joint_states'),
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        kick_controller_node
    ])
