# buttonHandler.py — Handler tombol OpenCR AROC26
# Semua logika state machine ada di TaskControl (main_task26)
# ButtonHandler hanya bertugas:
#   1. Terima tombol dari /robotis/open_cr/button
#   2. Teruskan ke TaskControl
#   3. Jalankan timer 20Hz untuk TaskControl.update()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from main_task26.task_control import TaskControl


class ButtonSoccerNode(Node):
    def __init__(self):
        super().__init__('button_soccer_node')

        # ── Task Control (state machine utama) ────────────────
        self.task = TaskControl(self)

        # ── Subscriber tombol ─────────────────────────────────
        self.create_subscription(
            String,
            '/robotis/open_cr/button',
            self._button_callback,
            10
        )

        # ── Timer 20Hz untuk TaskControl.update() ─────────────
        self.create_timer(0.05, self._loop)

        self.get_logger().info("=" * 45)
        self.get_logger().info("  ButtonSoccerNode AROC26 — Ready")
        self.get_logger().info("  USER  → toggle standup / sit")
        self.get_logger().info("  START → approach bola lalu berhenti")
        self.get_logger().info("  MODE  → (reserved)")
        self.get_logger().info("=" * 45)

    def _button_callback(self, msg: String):
        button = msg.data.strip()

        if button == "user":
            self.task.on_user_pressed()

        elif button == "start":
            self.task.on_start_pressed()

        elif button == "mode":
            self.get_logger().info("[Button] MODE — belum ada fungsi.")

    def _loop(self):
        """Timer 20Hz — delegasi ke TaskControl.update()"""
        self.task.update()


def main(args=None):
    rclpy.init(args=args)
    node = ButtonSoccerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
