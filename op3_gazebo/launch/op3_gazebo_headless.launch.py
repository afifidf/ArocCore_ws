# =============================================================================
# op3_gazebo_headless.launch.py
# =============================================================================
# Launch file untuk simulasi ROBOTIS OP3 di Gazebo TANPA GUI (headless)
# Cocok untuk testing di server atau environment tanpa display
#
# Cara menjalankan:
#   ros2 launch op3_gazebo op3_gazebo_headless.launch.py
# =============================================================================

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ==========================================================================
    # PATHS
    # ==========================================================================
    pkg_op3_gazebo = get_package_share_directory('op3_gazebo')
    pkg_op3_description = get_package_share_directory('op3_description')

    # ==========================================================================
    # LAUNCH ARGUMENTS
    # ==========================================================================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # ==========================================================================
    # ROBOT DESCRIPTION (URDF)
    # ==========================================================================
    urdf_file = os.path.join(pkg_op3_description, 'urdf', 'robotis_op3.urdf.xacro')
    
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', urdf_file
    ])

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # ==========================================================================
    # START GAZEBO (Headless - server only, no GUI)
    # ==========================================================================
    world_file = os.path.join(pkg_op3_gazebo, 'worlds', 'op3_empty.world')
    
    # Run Gazebo in headless mode (server only, -s flag)
    start_gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', '-s', world_file],  # -s = server only (headless)
        output='screen',
        shell=False
    )

    # ==========================================================================
    # ROBOT STATE PUBLISHER
    # ==========================================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # ==========================================================================
    # SPAWN ROBOT (delayed to wait for Gazebo)
    # ==========================================================================
    spawn_robot = TimerAction(
        period=5.0,  # Wait 5 seconds for Gazebo to start
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'robotis_op3',
                    '-topic', 'robot_description',
                    '-x', '0.0',
                    '-y', '0.0', 
                    '-z', '0.285',
                ],
                output='screen'
            )
        ]
    )

    # ==========================================================================
    # ROS-GAZEBO BRIDGE
    # ==========================================================================
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )

    # ==========================================================================
    # LAUNCH DESCRIPTION
    # ==========================================================================
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,

        # Start Gazebo (headless)
        start_gazebo,

        # Robot description
        robot_state_publisher,

        # Spawn robot (delayed)
        spawn_robot,

        # Bridge
        ros_gz_bridge,
    ])
