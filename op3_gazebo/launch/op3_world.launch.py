# =============================================================================
# op3_world.launch.py
# =============================================================================
# Launch file untuk simulasi ROBOTIS OP3 di Gazebo Sim (ROS2)
# Dikonversi dari ROS1: roslaunch op3_gazebo robotis_world.launch
#
# Cara menjalankan:
#   ros2 launch op3_gazebo op3_world.launch.py
# =============================================================================

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    ExecuteProcess, 
    TimerAction,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ==========================================================================
    # PATHS
    # ==========================================================================
    pkg_op3_gazebo = get_package_share_directory('op3_gazebo')
    pkg_op3_description = get_package_share_directory('op3_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Path untuk resource (mesh files)
    meshes_parent_path = os.path.dirname(pkg_op3_description)

    # ==========================================================================
    # LAUNCH ARGUMENTS
    # ==========================================================================
    paused_arg = DeclareLaunchArgument(
        'paused',
        default_value='true',
        description='Start simulation paused'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo with GUI'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # ==========================================================================
    # ENVIRONMENT VARIABLES
    # ==========================================================================
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=meshes_parent_path
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
    # GAZEBO SIM
    # ==========================================================================
    world_file = os.path.join(pkg_op3_gazebo, 'worlds', 'empty.world')
    
    # Start Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
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
    # SPAWN ROBOT
    # ==========================================================================
    # Spawn robot dengan posisi dan orientasi yang benar
    # Z = 0.24 (setengah tinggi robot agar kaki di tanah)
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'robotis_op3',
                    '-topic', 'robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.24',
                    '-R', '0.0',      # Roll
                    '-P', '0.0',      # Pitch  
                    '-Y', '0.0',      # Yaw
                ],
                output='screen'
            )
        ]
    )

    # ==========================================================================
    # ROS-GZ BRIDGE
    # ==========================================================================
    # Bridge untuk clock
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )
    
    # Bridge untuk joint states (dari Gazebo ke ROS)
    bridge_joint_states = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen'
    )
    
    # List of all joints for command bridge
    joints = [
        'head_pan', 'head_tilt',
        'l_sho_pitch', 'l_sho_roll', 'l_el',
        'r_sho_pitch', 'r_sho_roll', 'r_el',
        'l_hip_yaw', 'l_hip_roll', 'l_hip_pitch', 'l_knee', 'l_ank_pitch', 'l_ank_roll',
        'r_hip_yaw', 'r_hip_roll', 'r_hip_pitch', 'r_knee', 'r_ank_pitch', 'r_ank_roll',
    ]
    
    # Bridge untuk joint commands (dari ROS ke Gazebo)
    joint_bridge_args = [f'/model/robotis_op3/joint/{j}/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double' for j in joints]
    
    bridge_joint_cmds = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=joint_bridge_args,
        output='screen'
    )

    # ==========================================================================
    # INIT POSE (setelah robot spawn)
    # ==========================================================================
    init_pose = TimerAction(
        period=5.0,  # Tunggu robot spawn dulu
        actions=[
            Node(
                package='op3_gazebo',
                executable='set_init_pose.py',
                name='init_pose_publisher',
                output='screen',
            )
        ]
    )

    # ==========================================================================
    # LAUNCH DESCRIPTION
    # ==========================================================================
    return LaunchDescription([
        # Environment
        set_gz_resource_path,
        
        # Arguments
        paused_arg,
        gui_arg,
        use_sim_time_arg,

        # Gazebo
        gazebo,

        # Robot
        robot_state_publisher,
        spawn_robot,

        # Bridge
        bridge_clock,
        bridge_joint_states,
        bridge_joint_cmds,
        
        # Init pose
        init_pose,
    ])
