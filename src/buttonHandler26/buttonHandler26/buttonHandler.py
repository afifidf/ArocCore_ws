# loc: /home/afifi/aroc26/src/buttonHandler26/buttonHandler26/buttonHandler.py
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist


class ButtonSoccerNode(Node):

    def __init__(self):
        super().__init__('button_soccer_node')

        self.create_subscription(
            String,
            '/robotis/open_cr/button',
            self.button_callback,
            10
        )

        self.module_pub = self.create_publisher(
            String,
            '/robotis/enable_ctrl_module',
            10
        )

        self.walking_command_pub = self.create_publisher(
            String,
            '/robotis/walking/command',
            10
        )

        self.walking_vel_pub = self.create_publisher(
            Twist,
            '/robotis/walking/velocity',
            10
        )

        self.action_pub = self.create_publisher(
            Int32,
            '/robotis/action/page_num',
            10
        )

        # NEW → control head
        self.head_state_pub = self.create_publisher(
            String,
            '/head/state',
            10
        )

        self.is_walking = False

        self.get_logger().info("Button Soccer Node Ready")

    # ==========================================================
    def button_callback(self, msg):

        button = msg.data

        if button == "user":
            self.stop_walking()

            module_msg = String()
            module_msg.data = "action_module"
            self.module_pub.publish(module_msg)

            time.sleep(0.5)

            page = Int32()
            page.data = 15
            self.action_pub.publish(page)

            # Head OFF
            off = String()
            off.data = "off"
            self.head_state_pub.publish(off)

        elif button == "start":

            if not self.is_walking:
                self.start_walking()

                # Aktifkan head scan
                scan = String()
                scan.data = "scan"
                self.head_state_pub.publish(scan)

            else:
                self.stop_walking()

                off = String()
                off.data = "off"
                self.head_state_pub.publish(off)

    # ==========================================================
    def start_walking(self):

        module_msg = String()
        module_msg.data = "walking_module"
        self.module_pub.publish(module_msg)

        time.sleep(0.3)

        cmd = String()
        cmd.data = "start"
        self.walking_command_pub.publish(cmd)

        twist = Twist()
        twist.linear.x = 0.03
        self.walking_vel_pub.publish(twist)

        self.is_walking = True
        self.get_logger().info("Walking START + Head SCAN")

    def stop_walking(self):

        cmd = String()
        cmd.data = "stop"
        self.walking_command_pub.publish(cmd)

        self.is_walking = False
        self.get_logger().info("Walking STOP")


def main(args=None):
    rclpy.init(args=args)
    node = ButtonSoccerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()