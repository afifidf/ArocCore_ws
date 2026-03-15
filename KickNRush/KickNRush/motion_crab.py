# motion_crab.py — Hitung parameter crab walk (geser samping tanpa rotasi)
# Dipakai untuk fine-tune posisi kaki sebelum tendang (state ADJUST)
# Publisher ada di task_control.py, bukan di sini

from rclpy.node import Node

CRAB_Y_SPEED   = 0.020   # m — kecepatan normal
CRAB_Y_FAST    = 0.040   # m — kecepatan cepat (koreksi besar)
CRAB_Y_SLOW    = 0.010   # m — kecepatan lambat (fine-tune akhir)
CRAB_X_FORWARD = 0.0     # m — tidak ada komponen maju
CRAB_ANGLE     = 0.0     # deg — tidak ada rotasi (dikunci)
CRAB_PERIOD    = 0.55    # s — period satu langkah


class MotionCrab:
    """
    Hitung parameter walking untuk crab walk (geser murni, angle = 0).
    Berbeda dengan orbit: tidak ada rotasi, hanya geser samping.
    Mode kecepatan: 'normal', 'fast', 'slow'
    """

    def __init__(self, node: Node):
        self.node        = node
        self._direction  = "right"
        self._x          = CRAB_X_FORWARD
        self._y          = -CRAB_Y_SPEED   # default kanan
        self._angle      = CRAB_ANGLE
        self._period     = CRAB_PERIOD
        self._speed_mode = "normal"
        self.node.get_logger().info("[MotionCrab] Init — default: right")

    def set_direction(self, direction: str, speed_mode: str = "normal"):
        """
        Set arah dan kecepatan crab.
        direction: 'left' atau 'right'
        speed_mode: 'normal', 'fast', 'slow'
        """
        self._direction  = direction
        self._speed_mode = speed_mode

        y_speed = {"fast": CRAB_Y_FAST, "slow": CRAB_Y_SLOW}.get(speed_mode, CRAB_Y_SPEED)

        if direction == "right":
            self._y = -y_speed
        elif direction == "left":
            self._y = +y_speed
        else:
            self.node.get_logger().warn(f"[MotionCrab] Arah tidak dikenal: '{direction}'")
            return

        self._angle = CRAB_ANGLE  # selalu 0
        self.node.get_logger().info(
            f"[MotionCrab] → {direction} | speed={speed_mode} | y={self._y:.3f}m")

    def set_custom(self, x: float, y: float, period: float = CRAB_PERIOD):
        """Set parameter manual. Angle dikunci 0 — pakai MotionOrbit jika butuh rotasi."""
        self._x, self._y, self._angle, self._period = x, y, 0.0, period
        self.node.get_logger().info(f"[MotionCrab] Custom → x={x} y={y} period={period}")

    def get_params(self) -> tuple:
        """Return (x, y, angle, period)"""
        return self._x, self._y, self._angle, self._period

    def crab_step(self, direction: str, speed_mode: str = "normal") -> tuple:
        """Shortcut: set arah lalu return params"""
        self.set_direction(direction, speed_mode)
        return self.get_params()

    def estimate_direction_from_ball(self, ball_cx: float,
                                     frame_w: float = 640.0,
                                     deadband: float = 40.0) -> str | None:
        """
        Estimasi arah crab dari posisi bola di frame.
        Return: 'left', 'right', atau None jika sudah di tengah.
        """
        error = ball_cx - (frame_w / 2.0)
        if abs(error) < deadband:
            self.node.get_logger().info(
                f"[MotionCrab] Bola di tengah (error={error:.1f}px)")
            return None
        return "right" if error > 0 else "left"
