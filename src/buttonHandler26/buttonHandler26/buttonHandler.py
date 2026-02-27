import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist


class ButtonSoccerNode(Node):

    def __init__(self):
        super().__init__('button_soccer_node')

        # ==============================
        # Subscriber tombol OpenCR
        # ==============================
        self.button_sub = self.create_subscription(
            String,
            '/robotis/open_cr/button',
            self.button_callback,
            10
        )

        # ==============================
        # Publisher control module
        # ==============================
        self.module_pub = self.create_publisher(
            String,
            '/robotis/enable_ctrl_module',
            10
        )

        # ==============================
        # Publisher walking start/stop
        # ==============================
        self.walking_command_pub = self.create_publisher(
            String,
            '/robotis/walking/command',
            10
        )

        # ==============================
        # Publisher velocity (Twist)
        # ==============================
        self.walking_vel_pub = self.create_publisher(
            Twist,
            '/robotis/walking/velocity',
            10
        )

        # ==============================
        # Publisher action page (Init Pose)
        # ==============================
        self.action_pub = self.create_publisher(
            Int32,
            '/robotis/action/page_num',
            10
        )

        # ==============================
        # State variables
        # ==============================
        self.is_walking = False
        self.current_mode = "walking"  # bisa: "walking" atau "soccer"

        self.get_logger().info("Button Soccer Node Ready | Mode: Walking")

    # ==========================================================
    # BUTTON CALLBACK
    # ==========================================================
    def button_callback(self, msg):
        button = msg.data

        # ------------------------------
        # USER BUTTON → INIT POSE
        # ------------------------------
        if button == "user":
            self.get_logger().info("USER pressed → Init Pose")
            if self.is_walking:
                self.stop_walking()

            module_msg = String()
            module_msg.data = "action_module"
            self.module_pub.publish(module_msg)

            time.sleep(0.5)  # kasih waktu module aktif

            page = Int32()
            page.data = 15  # page init pose
            self.action_pub.publish(page)

            self.is_walking = False

        # ------------------------------
        # MODE BUTTON → TOGGLE WALKING / SOCCER
        # ------------------------------
        elif button == "mode":
            if self.current_mode == "walking":
                self.current_mode = "soccer"
                self.get_logger().info("MODE pressed → Soccer Mode")
                self.enable_soccer_mode()
            else:
                self.current_mode = "walking"
                self.get_logger().info("MODE pressed → Walking Mode")
                self.enable_walking_mode()

        # ------------------------------
        # START BUTTON → TOGGLE WALK
        # ------------------------------
        elif button == "start":
            if not self.is_walking:
                self.start_walking()
            else:
                self.stop_walking()

    # ==========================================================
    # ENABLE MODES
    # ==========================================================
    def enable_soccer_mode(self):
        if self.is_walking:
            self.stop_walking()

        module_msg = String()
        module_msg.data = "soccer_module"
        self.module_pub.publish(module_msg)

    def enable_walking_mode(self):
        if self.is_walking:
            self.stop_walking()

        module_msg = String()
        module_msg.data = "walking_module"
        self.module_pub.publish(module_msg)

    # ==========================================================
    # START WALKING
    # ==========================================================
    def start_walking(self):
        self.get_logger().info(f"Start Walking | Mode: {self.current_mode}")

        cmd_msg = String()
        cmd_msg.data = "start"
        self.walking_command_pub.publish(cmd_msg)

        twist = Twist()
        twist.linear.x = 0.03
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.walking_vel_pub.publish(twist)

        self.is_walking = True

    # ==========================================================
    # STOP WALKING
    # ==========================================================
    def stop_walking(self):
        self.get_logger().info("Stop Walking")

        cmd_msg = String()
        cmd_msg.data = "stop"
        self.walking_command_pub.publish(cmd_msg)

        self.is_walking = False


def main(args=None):
    rclpy.init(args=args)
    node = ButtonSoccerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()