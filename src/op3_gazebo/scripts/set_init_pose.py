#!/usr/bin/env python3
# =============================================================================
# set_init_pose.py
# =============================================================================
# Script untuk mengirim robot OP3 ke initial pose (berdiri/T-pose) di Gazebo Sim
# Dijalankan setelah robot di-spawn ke Gazebo
# =============================================================================

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time


class InitPosePublisher(Node):
    def __init__(self):
        super().__init__('init_pose_publisher')
        
        # Daftar semua joint dan posisi initial (bringup/standing pose)
        # Posisi dalam radian
        # Referensi dari ROS1 op3_manager init pose
        import math
        deg2rad = math.pi / 180.0
        
        self.joint_positions = {
            # Head - sedikit menunduk
            'head_pan': 0.0,
            'head_tilt': -10.0 * deg2rad,
            # Left Arm - tangan di samping badan
            'l_sho_pitch': -8.0 * deg2rad,
            'l_sho_roll': 17.0 * deg2rad,
            'l_el': 29.0 * deg2rad,
            # Right Arm - tangan di samping badan
            'r_sho_pitch': 8.0 * deg2rad,
            'r_sho_roll': -17.0 * deg2rad,
            'r_el': -29.0 * deg2rad,
            # Left Leg - sedikit ditekuk untuk stabilitas
            'l_hip_yaw': 0.0,
            'l_hip_roll': 0.0,
            'l_hip_pitch': -8.0 * deg2rad,
            'l_knee': 16.0 * deg2rad,
            'l_ank_pitch': -8.0 * deg2rad,
            'l_ank_roll': 0.0,
            # Right Leg - sedikit ditekuk untuk stabilitas
            'r_hip_yaw': 0.0,
            'r_hip_roll': 0.0,
            'r_hip_pitch': 8.0 * deg2rad,
            'r_knee': -16.0 * deg2rad,
            'r_ank_pitch': 8.0 * deg2rad,
            'r_ank_roll': 0.0,
        }
        
        # Create publishers for each joint
        self.publishers = {}
        for joint_name in self.joint_positions.keys():
            topic = f'/model/robotis_op3/joint/{joint_name}/cmd_pos'
            self.publishers[joint_name] = self.create_publisher(Float64, topic, 10)
            self.get_logger().info(f'Created publisher for {topic}')
        
        # Timer untuk publish
        self.timer = self.create_timer(0.1, self.publish_positions)
        self.count = 0
        self.max_count = 50  # Publish selama 5 detik (50 * 0.1s)
        
        self.get_logger().info('Init pose publisher started')

    def publish_positions(self):
        if self.count >= self.max_count:
            self.get_logger().info('Init pose complete!')
            self.timer.cancel()
            rclpy.shutdown()
            return
        
        for joint_name, position in self.joint_positions.items():
            msg = Float64()
            msg.data = position
            self.publishers[joint_name].publish(msg)
        
        self.count += 1
        if self.count % 10 == 0:
            self.get_logger().info(f'Publishing init pose... {self.count}/{self.max_count}')


def main(args=None):
    rclpy.init(args=args)
    node = InitPosePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
