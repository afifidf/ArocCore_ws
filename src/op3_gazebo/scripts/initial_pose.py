#!/usr/bin/env python3
# =============================================================================
# Initial Pose Publisher for ROBOTIS OP3 in Gazebo
# =============================================================================
# Script ini mengirim robot ke posisi bringup (berdiri siap)
# Digunakan setelah robot di-spawn ke Gazebo
# =============================================================================

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import math


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        
        # Parameter
        self.declare_parameter('config_file', '')
        self.declare_parameter('move_time', 2.0)
        
        # Publisher untuk joint trajectory controller
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # Subscriber untuk cek joint states
        self.joint_states_received = False
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Load config
        self.load_config()
        
        # Timer untuk publish initial pose setelah sistem siap
        self.timer = self.create_timer(2.0, self.publish_initial_pose)
        self.pose_published = False
        
        self.get_logger().info('Initial Pose Publisher initialized')
        self.get_logger().info('Waiting for joint states...')

    def joint_state_callback(self, msg):
        if not self.joint_states_received:
            self.joint_states_received = True
            self.get_logger().info('Joint states received, system ready')

    def load_config(self):
        """Load initial pose configuration from YAML"""
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        
        if not config_file:
            # Default config path
            try:
                pkg_share = get_package_share_directory('op3_gazebo')
                config_file = os.path.join(pkg_share, 'config', 'initial_pose.yaml')
            except Exception:
                config_file = ''
        
        # Default values (bringup pose)
        self.move_time = self.get_parameter('move_time').get_parameter_value().double_value
        
        # Default joint positions (in radians) - bringup pose
        self.joint_positions = {
            # Right Arm
            'r_sho_pitch': math.radians(15),
            'r_sho_roll': math.radians(-45),
            'r_el': math.radians(45),
            # Left Arm
            'l_sho_pitch': math.radians(-15),
            'l_sho_roll': math.radians(45),
            'l_el': math.radians(-45),
            # Right Leg
            'r_hip_yaw': 0.0,
            'r_hip_roll': 0.0,
            'r_hip_pitch': math.radians(70),
            'r_knee': math.radians(-142),
            'r_ank_pitch': math.radians(-70),
            'r_ank_roll': 0.0,
            # Left Leg
            'l_hip_yaw': 0.0,
            'l_hip_roll': 0.0,
            'l_hip_pitch': math.radians(-70),
            'l_knee': math.radians(142),
            'l_ank_pitch': math.radians(70),
            'l_ank_roll': 0.0,
            # Head
            'head_pan': 0.0,
            'head_tilt': math.radians(-10),
        }
        
        # Try to load from file
        if config_file and os.path.exists(config_file):
            try:
                with open(config_file, 'r') as f:
                    config = yaml.safe_load(f)
                    
                if 'initial_pose' in config:
                    pose_config = config['initial_pose']
                    
                    if 'move_time' in pose_config:
                        self.move_time = pose_config['move_time']
                    
                    if 'joints' in pose_config:
                        self.joint_positions.update(pose_config['joints'])
                        
                self.get_logger().info(f'Loaded config from: {config_file}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load config: {e}, using defaults')
        else:
            self.get_logger().info('Using default initial pose configuration')

    def publish_initial_pose(self):
        """Publish initial pose to joint trajectory controller"""
        
        if self.pose_published:
            return
            
        if not self.joint_states_received:
            self.get_logger().info('Waiting for joint states before publishing initial pose...')
            return
        
        # Create trajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Joint names
        trajectory_msg.joint_names = list(self.joint_positions.keys())
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = list(self.joint_positions.values())
        point.velocities = [0.0] * len(self.joint_positions)
        point.time_from_start = Duration(sec=int(self.move_time), 
                                          nanosec=int((self.move_time % 1) * 1e9))
        
        trajectory_msg.points.append(point)
        
        # Publish
        self.trajectory_pub.publish(trajectory_msg)
        self.pose_published = True
        
        self.get_logger().info('='*50)
        self.get_logger().info('Initial pose published!')
        self.get_logger().info(f'Move time: {self.move_time} seconds')
        self.get_logger().info(f'Number of joints: {len(self.joint_positions)}')
        self.get_logger().info('='*50)
        
        # Cancel timer
        self.timer.cancel()
        
        # Shutdown after a delay
        self.create_timer(self.move_time + 1.0, self.shutdown_node)

    def shutdown_node(self):
        self.get_logger().info('Initial pose complete. Shutting down...')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
