# orbit.py — Hitung parameter orbit mengelilingi bola
# Orbit = y_move (geser samping) + angle_move (rotasi badan) bersamaan
# Publisher WalkingParam ada di node yang pakai kelas ini
# Angle dikembalikan dalam DERAJAT — konversi ke radian di task_control

from rclpy.node import Node

# ── Tuning parameter ──────────────────────────────────────────
ORBIT_Y_SPEED     = 0.030   # m   — kecepatan geser samping
ORBIT_ANGLE_SPEED = 7.0     # deg — rotasi badan per langkah
ORBIT_X_FORWARD   = 0.010   # m   — maju kecil agar radius tidak membesar
ORBIT_PERIOD      = 0.55    # s   — periode satu langkah


class orbitalMotion:
    """
    Hitung parameter walking untuk orbit mengelilingi bola.

    Prinsip:
      Orbit KANAN → y negatif (geser kanan) + angle positif (putar kiri)
      Orbit KIRI  → y positif (geser kiri)  + angle negatif (putar kanan)

    Angle dikembalikan dalam DERAJAT.
    Konversi ke radian dilakukan di task_control sebelum publish WalkingParam:
        param.angle_move_amplitude = angle_deg * math.pi / 180.0
    """

    def __init__(self, node: Node):
        self.node       = node
        self._direction = "right"
        self._x         = ORBIT_X_FORWARD
        self._y         = -ORBIT_Y_SPEED     # default: kanan
        self._angle     = ORBIT_ANGLE_SPEED  # default: kanan
        self._period    = ORBIT_PERIOD
        self.node.get_logger().info("[orbitalMotion] Init — default: right")

    def set_direction(self, direction: str):
        """
        Set arah orbit: 'left' atau 'right'

        Orbit kanan: y negatif + angle positif
        Orbit kiri : y positif + angle negatif
        """
        self._direction = direction
        if direction == "right":
            self._y     = -ORBIT_Y_SPEED
            self._angle = +ORBIT_ANGLE_SPEED
        elif direction == "left":
            self._y     = +ORBIT_Y_SPEED
            self._angle = -ORBIT_ANGLE_SPEED
        else:
            self.node.get_logger().warn(
                f"[orbitalMotion] Arah tidak dikenal: '{direction}'. Gunakan 'left'/'right'.")
            return
        self.node.get_logger().info(
            f"[orbitalMotion] → {direction} | y={self._y:.3f}m | angle={self._angle:.1f}deg")

    def set_custom(self, x: float, y: float, angle: float, period: float = ORBIT_PERIOD):
        """
        Set parameter orbit manual — untuk fine-tuning di lapangan.
        angle dalam DERAJAT.
        """
        self._x      = x
        self._y      = y
        self._angle  = angle
        self._period = period
        self.node.get_logger().info(
            f"[orbitalMotion] Custom → x={x} y={y} angle={angle}deg period={period}s")

    def get_params(self) -> tuple:
        """
        Return (x, y, angle_deg, period)
        angle dalam DERAJAT — konversi ke radian di task_control!
        """
        return self._x, self._y, self._angle, self._period

    def orbit_step(self, direction: str) -> tuple:
        """Shortcut: set arah lalu langsung return params."""
        self.set_direction(direction)
        return self.get_params()
