from rclpy.node import Node

# ── Tuning parameter ──────────────────────────────────────────
APPROACH_X_SPEED    = 0.030   # m — kecepatan maju mendekati bola (jauh)
APPROACH_X_SLOW     = 0.015   # m — kecepatan maju saat bola mulai dekat
APPROACH_Y_SPEED    = 0.000   # m — tidak geser samping
APPROACH_ANGLE      = 0.000   # rad — tidak putar badan (WalkingParam pakai radian!)

# Ukuran bbox bola (lebar, px) sebagai estimasi jarak
# Makin besar angka → bola makin dekat ke kamera
BALL_WIDTH_STOP     = 120     # px — berhenti di sini (~20-30 cm dari bola)
BALL_WIDTH_SLOWDOWN = 80      # px — mulai pelan dari sini


class MotionApproach:
    def __init__(self, node: Node):
        self.node = node
        self.node.get_logger().info(
            f"[MotionApproach] Init | "
            f"stop={BALL_WIDTH_STOP}px | slowdown={BALL_WIDTH_SLOWDOWN}px"
        )

    def get_params(self, ball_width: float) -> tuple:
        if ball_width >= BALL_WIDTH_SLOWDOWN:
            x_speed = APPROACH_X_SLOW    # bola dekat → pelan
        else:
            x_speed = APPROACH_X_SPEED   # bola jauh  → penuh

        return x_speed, APPROACH_Y_SPEED, APPROACH_ANGLE

    def is_close_enough(self, ball_width: float) -> bool:
        result = ball_width >= BALL_WIDTH_STOP
        if result:
            self.node.get_logger().info(
                f"[MotionApproach] Bola cukup dekat! "
                f"ball_width={ball_width:.0f}px >= {BALL_WIDTH_STOP}px → STOP"
            )
        return result

    def set_stop_threshold(self, px: int):
        """Ubah threshold berhenti saat runtime (untuk tuning di lapangan)."""
        global BALL_WIDTH_STOP
        BALL_WIDTH_STOP = px
        self.node.get_logger().info(
            f"[MotionApproach] Stop threshold diubah → {px}px")
