# main.py — Entry point node ROS2 KickNRush
# Inisialisasi TaskControl dan jalankan loop 20 Hz
# Jalankan: ros2 launch KickNRush KickNRush.launch.py
# Pastikan headControl26 sudah jalan (vision + head tracking)

import rclpy
from rclpy.node import Node
from ._task_control import TaskControl

LOOP_HZ = 20  # frekuensi update (Hz)


class KickNRushNode(Node):
    """Node utama KickNRush. Semua logika ada di TaskControl."""

    def __init__(self):
        super().__init__('kicknrush_node')
        self.get_logger().info("=" * 50)
        self.get_logger().info("  KickNRush — AROC26 | Orbit | Crab | Kick")
        self.get_logger().info("=" * 50)
        self.get_logger().info("Tekan START di OpenCR untuk mulai.")

        self.task  = TaskControl(self)
        self.timer = self.create_timer(1.0 / LOOP_HZ, self._loop)
        self.get_logger().info(f"[KickNRush] Loop @ {LOOP_HZ} Hz ✅")

    def _loop(self):
        """Loop 20 Hz — delegasi ke TaskControl.update()"""
        try:
            self.task.update()
        except Exception as e:
            self.get_logger().error(f"[KickNRush] Error: {e}")


def main():
    rclpy.init()
    node = KickNRushNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[KickNRush] Dihentikan.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
