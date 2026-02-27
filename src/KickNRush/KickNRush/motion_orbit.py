# motion_orbit.py — Hitung parameter orbit mengelilingi bola
# Orbit = y_move (geser) + angle_move (rotasi) bersamaan
# Publisher ada di task_control.py, bukan di sini

from rclpy.node import Node

ORBIT_Y_SPEED     = 0.030   # m — kecepatan geser samping
ORBIT_ANGLE_SPEED = 7.0     # deg — rotasi per langkah
ORBIT_X_FORWARD   = 0.010   # m — maju kecil agar radius tidak membesar
ORBIT_PERIOD      = 0.55    # s — period satu langkah


class MotionOrbit:
    """
    Hitung parameter walking untuk orbit mengelilingi bola.
    Orbit kanan: y negatif + angle positif
    Orbit kiri : y positif + angle negatif
    Angle return dalam DERAJAT — konversi ke rad di task_control.
    """

    def __init__(self, node: Node):
        self.node     = node
        self._direction = "right"
        self._x       = ORBIT_X_FORWARD
        self._y       = -ORBIT_Y_SPEED    # default kanan
        self._angle   = ORBIT_ANGLE_SPEED # default kanan
        self._period  = ORBIT_PERIOD
        self.node.get_logger().info("[MotionOrbit] Init — default: right")

    def set_direction(self, direction: str):
        """Set arah orbit: 'left' atau 'right'"""
        self._direction = direction
        if direction == "right":
            self._y     = -ORBIT_Y_SPEED
            self._angle = +ORBIT_ANGLE_SPEED
        elif direction == "left":
            self._y     = +ORBIT_Y_SPEED
            self._angle = -ORBIT_ANGLE_SPEED
        else:
            self.node.get_logger().warn(f"[MotionOrbit] Arah tidak dikenal: '{direction}'")
            return
        self.node.get_logger().info(
            f"[MotionOrbit] → {direction} | y={self._y:.3f}m angle={self._angle:.1f}°")

    def set_custom(self, x: float, y: float, angle: float, period: float = ORBIT_PERIOD):
        """Set parameter orbit manual untuk fine-tuning"""
        self._x, self._y, self._angle, self._period = x, y, angle, period
        self.node.get_logger().info(
            f"[MotionOrbit] Custom → x={x} y={y} angle={angle} period={period}")

    def get_params(self) -> tuple:
        """Return (x, y, angle_deg, period) — angle dalam DERAJAT"""
        return self._x, self._y, self._angle, self._period

    def orbit_step(self, direction: str) -> tuple:
        """Shortcut: set arah lalu return params"""
        self.set_direction(direction)
        return self.get_params()
