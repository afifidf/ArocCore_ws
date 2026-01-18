# =============================================================================
# op3_gazebo.launch.py
# =============================================================================
# Launch file untuk simulasi ROBOTIS OP3 di Gazebo (ROS2)
#
# Cara menjalankan:
#   ros2 launch op3_gazebo op3_gazebo.launch.py
#   ros2 launch op3_gazebo op3_gazebo.launch.py gui:=true
#   ros2 launch op3_gazebo op3_gazebo.launch.py world:=custom_world.world
# =============================================================================

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ==========================================================================
    # PATHS
    # ==========================================================================
    pkg_op3_gazebo = get_package_share_directory('op3_gazebo')
    pkg_op3_description = get_package_share_directory('op3_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # ==========================================================================
    # LAUNCH ARGUMENTS
    # ==========================================================================
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo with GUI'
    )

    paused_arg = DeclareLaunchArgument(
        'paused',
        default_value='false',
        description='Start simulation paused'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_op3_gazebo, 'worlds', 'op3_empty.world'),
        description='World file to load'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    initial_pose_arg = DeclareLaunchArgument(
        'initial_pose',
        default_value='true',
        description='Send robot to initial bringup pose'
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
    # GAZEBO SIMULATION
    # ==========================================================================
    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 ', LaunchConfiguration('world')],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ==========================================================================
    # SPAWN ROBOT
    # ==========================================================================
    # Spawn robot ke Gazebo pada posisi z=0.285 (agar kaki menyentuh tanah)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robotis_op3',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.285',  # Tinggi spawn agar kaki di tanah
        ],
        output='screen'
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
    # ROS-GAZEBO BRIDGE
    # ==========================================================================
    # Bridge untuk topik antara Gazebo dan ROS2
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # IMU
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Camera
            '/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # Joint states (bidirectional)
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/imu', '/robotis/open_cr/imu'),
            ('/image_raw', '/robotis/camera/image_raw'),
        ],
        output='screen'
    )

    # ==========================================================================
    # JOINT STATE BROADCASTER (ros2_control)
    # ==========================================================================
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # ==========================================================================
    # JOINT TRAJECTORY CONTROLLER (ros2_control)
    # ==========================================================================
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # ==========================================================================
    # INITIAL POSE
    # ==========================================================================
    # Kirim robot ke initial pose setelah controller siap
    initial_pose_node = Node(
        package='op3_gazebo',
        executable='initial_pose.py',
        name='initial_pose_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'config_file': os.path.join(pkg_op3_gazebo, 'config', 'initial_pose.yaml')},
            {'move_time': 2.0},
        ],
    )

    # ==========================================================================
    # LAUNCH DESCRIPTION
    # ==========================================================================
    return LaunchDescription([
        # Arguments
        gui_arg,
        paused_arg,
        world_arg,
        use_sim_time_arg,
        initial_pose_arg,

        # Gazebo
        gazebo,

        # Robot
        robot_state_publisher,
        spawn_robot,

        # Bridge
        ros_gz_bridge,

        # Controllers (di-comment dulu jika belum setup ros2_control di URDF)
        # joint_state_broadcaster_spawner,
        # joint_trajectory_controller_spawner,

        # Initial pose (di-comment dulu sampai controller siap)
        # initial_pose_node,
    ])
