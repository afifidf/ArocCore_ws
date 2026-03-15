# goal_alignment.py — Deteksi apakah robot sudah sejajar ke gawang
# Fusion dua sensor: IMU yaw (quaternion) + YOLO gawang
# Mode: DUAL (keduanya), IMU only, YOLO only (fallback otomatis)
# Yaw diambil dari orientation quaternion — tidak drift seperti integrasi gyro

import math
import time
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion

IMU_TOLERANCE        = 8.0   # deg — toleransi yaw IMU
GOAL_PIXEL_TOLERANCE = 60    # px  — toleransi posisi gawang di frame
FRAME_W              = 640   # px  — lebar frame kamera
GOAL_DETECT_TIMEOUT  = 2.0  # s   — timeout gawang tidak terdeteksi
YAW_FILTER_ALPHA     = 0.15 # low-pass filter yaw (makin kecil = makin smooth)
ALIGN_CONFIRM_COUNT  = 5    # jumlah frame konsisten sebelum dinyatakan aligned


class GoalAlignment:
    """
    Cek apakah robot sudah menghadap gawang menggunakan fusion IMU + YOLO.
    Subscribe: /robotis/open_cr/imu, /obj_detect_goal
    Publish  : /goal_alignment/status, /goal_alignment/debug
    """

    def __init__(self, node: Node):
        self.node = node

        # State IMU (quaternion-based)
        self._yaw_raw      = 0.0   # yaw absolut dari quaternion (deg)
        self._yaw_ref      = None  # referensi yaw saat reset_yaw() dipanggil
        self._yaw_filtered = 0.0   # yaw error setelah low-pass filter
        self._imu_available = False

        # State YOLO gawang
        self._goal_cx         = None
        self._goal_cy         = None
        self._goal_width      = None
        self._last_goal_time  = 0.0
        self._goal_available  = False

        # Confirm counter & arah orbit saran
        self._align_count         = 0
        self._suggested_direction = "right"

        # Subscriber
        node.create_subscription(Imu, '/robotis/open_cr/imu', self._imu_callback, 10)
        node.create_subscription(String, '/obj_detect_goal', self._goal_callback, 10)

        # Publisher debug
        self._pub_status = node.create_publisher(String, '/goal_alignment/status', 10)
        self._pub_debug  = node.create_publisher(String, '/goal_alignment/debug', 10)

        self.node.get_logger().info("[GoalAlignment] Init — menunggu IMU & gawang")

    # ── CALLBACK ──────────────────────────────────────────────────────────────

    def _imu_callback(self, msg: Imu):
        """
        Baca yaw dari orientation quaternion (tidak drift).
        Hitung error relatif terhadap _yaw_ref (set saat reset_yaw()).
        Terapkan low-pass filter untuk redam noise.
        """
        q = [msg.orientation.x, msg.orientation.y,
             msg.orientation.z, msg.orientation.w]

        # Cek apakah quaternion valid (bukan semua nol)
        if all(v == 0.0 for v in q):
            return

        _, _, yaw_rad = euler_from_quaternion(q)
        self._yaw_raw = math.degrees(yaw_rad)   # simpan yaw absolut (deg)

        # Jika belum ada referensi, set otomatis dari nilai pertama
        if self._yaw_ref is None:
            self._yaw_ref = self._yaw_raw
            self.node.get_logger().info(
                f"[GoalAlignment] Yaw ref auto-set = {self._yaw_ref:.2f}°")

        # Hitung error relatif terhadap referensi
        yaw_error = self._yaw_raw - self._yaw_ref

        # Normalisasi ke range -180 ~ +180 deg
        if yaw_error > 180.0:
            yaw_error -= 360.0
        elif yaw_error < -180.0:
            yaw_error += 360.0

        # Low-pass filter
        self._yaw_filtered = (YAW_FILTER_ALPHA * yaw_error
                              + (1.0 - YAW_FILTER_ALPHA) * self._yaw_filtered)
        self._imu_available = True

    def _goal_callback(self, msg: String):
        """Parse pesan gawang 'cx,cy,w,h' dari /obj_detect_goal"""
        data = msg.data.strip()
        if not data:
            self._goal_available = False
            return
        try:
            parts = data.split(',')
            if len(parts) >= 2:
                self._goal_cx    = float(parts[0])
                self._goal_cy    = float(parts[1])
                self._goal_width = float(parts[2]) if len(parts) >= 3 else None
                self._last_goal_time = time.time()
                self._goal_available = True
        except (ValueError, IndexError):
            self._goal_available = False
            self.node.get_logger().warn("[GoalAlignment] Gagal parse: " + data)

    # ── PUBLIC METHOD ─────────────────────────────────────────────────────────

    def reset_yaw(self):
        """
        Reset referensi yaw ke nilai quaternion saat ini.
        Panggil saat robot berdiri menghadap gawang (tombol START atau setelah tendang).
        Setelah reset, yaw_error = 0 = arah robot saat ini.
        """
        self._yaw_ref      = self._yaw_raw   # simpan yaw absolut saat ini sebagai referensi
        self._yaw_filtered = 0.0
        self._align_count  = 0
        self.node.get_logger().info(
            f"[GoalAlignment] Yaw reset. Ref={self._yaw_ref:.2f}° (dari quaternion)")

    def is_goal_visible(self) -> bool:
        """True jika gawang terdeteksi dalam GOAL_DETECT_TIMEOUT detik terakhir"""
        return self._goal_available and (time.time() - self._last_goal_time < GOAL_DETECT_TIMEOUT)

    def get_yaw_error(self) -> float:
        """Return yaw error (deg). Positif=orbit kanan, negatif=orbit kiri"""
        return self._yaw_filtered

    def get_goal_pixel_error(self) -> float | None:
        """Return error posisi gawang dari tengah frame (px). None jika tidak terdeteksi"""
        if not self.is_goal_visible() or self._goal_cx is None:
            return None
        return self._goal_cx - (FRAME_W / 2.0)

    def get_orbit_direction(self) -> str:
        """
        Tentukan arah orbit otomatis dari fusion sensor.
        DUAL: IMU 60% + YOLO 40% | IMU only | YOLO only
        Return: 'left' atau 'right'
        """
        yaw_error  = self._yaw_filtered
        goal_error = self.get_goal_pixel_error()

        if self._imu_available and self.is_goal_visible() and goal_error is not None:
            # Mode DUAL — fusion berbobot
            yaw_from_goal = goal_error * (60.0 / FRAME_W)
            fused_error   = 0.6 * yaw_error + 0.4 * yaw_from_goal
            self._suggested_direction = "right" if fused_error > 0 else "left"
            self.node.get_logger().info(
                f"[GoalAlignment][DUAL] yaw={yaw_error:.1f}° goal={goal_error:.1f}px "
                f"fused={fused_error:.1f}° → {self._suggested_direction}")

        elif self._imu_available:
            # Mode IMU only
            self._suggested_direction = "right" if yaw_error > 0 else "left"
            self.node.get_logger().info(
                f"[GoalAlignment][IMU] yaw={yaw_error:.1f}° → {self._suggested_direction}")

        elif self.is_goal_visible() and goal_error is not None:
            # Mode YOLO only
            self._suggested_direction = "right" if goal_error > 0 else "left"
            self.node.get_logger().info(
                f"[GoalAlignment][YOLO] goal={goal_error:.1f}px → {self._suggested_direction}")

        else:
            self.node.get_logger().warn(
                "[GoalAlignment] Tidak ada sensor! Pakai arah terakhir: "
                + self._suggested_direction)

        return self._suggested_direction

    def is_aligned(self) -> bool:
        """
        True jika robot sudah sejajar ke gawang (AND logic, terkonfirmasi N frame).
        imu_ok : |yaw| < IMU_TOLERANCE
        yolo_ok: |goal_px| < GOAL_PIXEL_TOLERANCE (jika gawang terlihat)
        """
        yaw_error  = abs(self._yaw_filtered)
        goal_error = self.get_goal_pixel_error()

        imu_ok  = (not self._imu_available) or (yaw_error < IMU_TOLERANCE)
        yolo_ok = (abs(goal_error) < GOAL_PIXEL_TOLERANCE
                   if (self.is_goal_visible() and goal_error is not None) else True)

        aligned_now = imu_ok and yolo_ok
        self._align_count = self._align_count + 1 if aligned_now else 0
        confirmed = self._align_count >= ALIGN_CONFIRM_COUNT

        # Publish status & debug
        status_msg      = String()
        status_msg.data = "aligned" if confirmed else "not_aligned"
        self._pub_status.publish(status_msg)

        mode = ("DUAL" if (self._imu_available and self.is_goal_visible())
                else ("IMU" if self._imu_available else "YOLO"))
        goal_str = f"{goal_error:.1f}px" if goal_error is not None else "N/A"

        debug_msg      = String()
        debug_msg.data = (f"[{mode}] yaw={self._yaw_filtered:.1f}°(tol={IMU_TOLERANCE}°) | "
                          f"goal={goal_str}(tol={GOAL_PIXEL_TOLERANCE}px) | "
                          f"imu={imu_ok} yolo={yolo_ok} | "
                          f"count={self._align_count}/{ALIGN_CONFIRM_COUNT} | OK={confirmed}")
        self._pub_debug.publish(debug_msg)

        if confirmed:
            self.node.get_logger().info(f"[GoalAlignment] ✅ SEJAJAR! {debug_msg.data}")
        else:
            self.node.get_logger().debug(debug_msg.data)

        return confirmed

    def get_sensor_status(self) -> dict:
        """Return dict status semua sensor untuk debugging eksternal"""
        goal_error = self.get_goal_pixel_error()
        return {
            "imu_available"   : self._imu_available,
            "yaw_filtered_deg": round(self._yaw_filtered, 2),
            "imu_ok"          : abs(self._yaw_filtered) < IMU_TOLERANCE,
            "goal_visible"    : self.is_goal_visible(),
            "goal_cx"         : self._goal_cx,
            "goal_pixel_error": goal_error,
            "yolo_ok"         : (abs(goal_error) < GOAL_PIXEL_TOLERANCE
                                 if goal_error is not None else None),
            "align_count"     : self._align_count,
            "confirm_needed"  : ALIGN_CONFIRM_COUNT,
            "mode"            : ("DUAL" if (self._imu_available and self.is_goal_visible())
                                 else ("IMU" if self._imu_available else "YOLO")),
        }
