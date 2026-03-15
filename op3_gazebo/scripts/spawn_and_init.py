#!/usr/bin/env python3
# =============================================================================
# spawn_and_init.py
# =============================================================================
# Script untuk spawn robot OP3 dan langsung set ke initial pose
# Menggunakan Gazebo service untuk pause/unpause simulasi
# =============================================================================

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from ros_gz_interfaces.srv import ControlWorld
from geometry_msgs.msg import Pose
import subprocess
import time
import math


class SpawnAndInit(Node):
    def __init__(self):
        super().__init__('spawn_and_init')
        
        deg2rad = math.pi / 180.0
        
        # Initial pose untuk berdiri - tangan di samping badan
        self.joint_positions = {
            # Head
            'head_pan': 0.0,
            'head_tilt': 0.0,
            # Left Arm - tangan di samping
            'l_sho_pitch': 0.0,
            'l_sho_roll': 20.0 * deg2rad,
            'l_el': -50.0 * deg2rad,
            # Right Arm - tangan di samping  
            'r_sho_pitch': 0.0,
            'r_sho_roll': -20.0 * deg2rad,
            'r_el': 50.0 * deg2rad,
            # Left Leg - lurus berdiri
            'l_hip_yaw': 0.0,
            'l_hip_roll': 0.0,
            'l_hip_pitch': 0.0,
            'l_knee': 0.0,
            'l_ank_pitch': 0.0,
            'l_ank_roll': 0.0,
            # Right Leg - lurus berdiri
            'r_hip_yaw': 0.0,
            'r_hip_roll': 0.0,
            'r_hip_pitch': 0.0,
            'r_knee': 0.0,
            'r_ank_pitch': 0.0,
            'r_ank_roll': 0.0,
        }
        
        # Publishers untuk joint commands
        self.publishers = {}
        for joint_name in self.joint_positions.keys():
            topic = f'/model/robotis_op3/joint/{joint_name}/cmd_pos'
            self.publishers[joint_name] = self.create_publisher(Float64, topic, 10)
        
        self.get_logger().info('Waiting for Gazebo...')
        time.sleep(2.0)
        
        # Langsung publish joint positions terus menerus
        self.timer = self.create_timer(0.02, self.publish_positions)  # 50Hz
        self.get_logger().info('Publishing joint positions...')

    def publish_positions(self):
        for joint_name, position in self.joint_positions.items():
            msg = Float64()
            msg.data = position
            self.publishers[joint_name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SpawnAndInit()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
