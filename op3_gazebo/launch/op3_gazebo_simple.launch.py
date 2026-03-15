# =============================================================================
# op3_gazebo_simple.launch.py
# =============================================================================
# Launch file SEDERHANA untuk simulasi ROBOTIS OP3 di Gazebo (ROS2)
# Versi ini lebih simple, tanpa ros2_control (untuk testing awal)
#
# Cara menjalankan:
#   ros2 launch op3_gazebo op3_gazebo_simple.launch.py
# =============================================================================

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable
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
    
    # Path ke folder yang berisi meshes (parent dari meshes folder)
    # Gazebo akan mencari "op3_description/meshes/xxx.stl" di dalam path ini
    meshes_parent_path = os.path.dirname(pkg_op3_description)

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
    # START GAZEBO (Ignition Gazebo / Gazebo Sim)
    # ==========================================================================
    world_file = os.path.join(pkg_op3_gazebo, 'worlds', 'op3_empty.world')
    
    # Coba Gazebo Harmonic/Fortress (gz sim) 
    start_gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_file],
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
        period=3.0,  # Wait 3 seconds for Gazebo to start
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
    # ROS-GAZEBO BRIDGE (for clock and sensors)
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
    # CONTROLLER SPAWNERS (ros2_control)
    # ==========================================================================
    # Spawn joint_state_broadcaster
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,  # Wait for robot to be spawned
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen',
            )
        ]
    )

    # Spawn joint_trajectory_controller
    joint_trajectory_controller_spawner = TimerAction(
        period=6.0,  # Wait for joint_state_broadcaster
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
                output='screen',
            )
        ]
    )

    # ==========================================================================
    # JOINT STATE PUBLISHER GUI (untuk kontrol manual joint - DISABLED)
    # Tidak diperlukan karena sudah ada joint_state_broadcaster dari ros2_control
    # ==========================================================================
    # joint_state_publisher_gui = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     output='screen',
    # )

    # ==========================================================================
    # RVIZ (optional - untuk visualisasi)
    # ==========================================================================
    rviz_config = os.path.join(pkg_op3_description, 'rviz', 'op3.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # ==========================================================================
    # ENVIRONMENT VARIABLES untuk Gazebo resource paths
    # ==========================================================================
    # Set GZ_SIM_RESOURCE_PATH agar Gazebo bisa menemukan meshes
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=meshes_parent_path
    )

    # ==========================================================================
    # LAUNCH DESCRIPTION
    # ==========================================================================
    return LaunchDescription([
        # Environment variables (HARUS di awal!)
        set_gz_resource_path,
        
        # Arguments
        use_sim_time_arg,

        # Start Gazebo
        start_gazebo,

        # Robot description
        robot_state_publisher,

        # Spawn robot (delayed)
        spawn_robot,

        # Bridge
        ros_gz_bridge,

        # Controllers (ros2_control)
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,

        # Visualization
        # rviz,  # Uncomment jika mau RViz juga
    ])
