# mainHeadControl.py — Head tracking node ROS2 untuk robot OP3 AROC26
# Subscribe /obj_detect → EKF filter → PID → publish JointState ke head
# Mode: TRACK (bola terdeteksi) | SCAN (bola hilang, kepala sweep kiri-kanan)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from .motionPID import PIDControl
from .ballEKF import BallEKF
import time

HEAD_PAN  = 0
HEAD_TILT = 1

# ── Tuning PID ────────────────────────────────────────────────
# TILT
TILT_KP   = 1.2    # gain proportional
TILT_KI   = 0.8    # gain integral
TILT_KD   = 1.2    # gain derivative
TILT_TI   = 10.0   # periode integral (ms) — anti windup timing
TILT_TD   = 10.0   # periode derivative (ms)
TILT_MIN  = -1.2   # output min (rad)
TILT_MAX  =  0.0   # output max (rad)
TILT_SP   = 240.0  # setpoint (tengah frame vertikal, px)

# PAN
PAN_KP    = 0.75
PAN_KI    = 0.0
PAN_KD    = 0.0
PAN_TI    = 10.0
PAN_TD    = 10.0
PAN_MIN   = -1.2
PAN_MAX   =  1.2
PAN_SP    = 320.0  # setpoint (tengah frame horizontal, px)

# ── Frame size — harus sama dengan vision.py ──────────────────
FRAME_W = 640
FRAME_H = 480

# ── Threshold deadband (px) — di bawah ini PID tidak update ──
DEADBAND_X = 8
DEADBAND_Y = 8

# ── Scan parameter ────────────────────────────────────────────
SCAN_STEP    = 0.05   # rad per tick saat sweep
SCAN_TILT    = -0.4   # posisi tilt saat scan
SCAN_TIMEOUT = 0.7    # s — bola hilang lebih dari ini → masuk scan mode

# ── Joint limit ───────────────────────────────────────────────
PAN_LIMIT  = 1.2
TILT_LOWER = -1.2
TILT_UPPER =  0.0


class HeadControlNode(Node):
    """
    Node head tracking robot OP3.

    Alur:
      /obj_detect → EKF update → PID calculate → JointState publish
                                              ↓
      Jika bola hilang > SCAN_TIMEOUT → scan_head() (sweep kiri-kanan)

    Topic subscribe:
      /obj_detect   → posisi bola (cx,cy) dari vision.py
      /head/state   → "scan" aktifkan head | "off" nonaktifkan

    Topic publish:
      /robotis/head_control/set_joint_states → posisi servo kepala
      /robotis/enable_ctrl_module            → aktifkan head_control_module
      /vision/ball_measurement               → posisi bola raw (debug)
      /vision/ball_ekf                       → posisi bola setelah EKF (debug)
    """

    def __init__(self):
        super().__init__('headcontrol_node')

        # ── Publisher ─────────────────────────────────────────
        self.module_pub   = self.create_publisher(String,    '/robotis/enable_ctrl_module', 10)
        self.head_pub     = self.create_publisher(JointState, '/robotis/head_control/set_joint_states', 10)
        self.ball_meas_pub = self.create_publisher(Vector3,   '/vision/ball_measurement', 10)
        self.ball_ekf_pub  = self.create_publisher(Vector3,   '/vision/ball_ekf', 10)

        # ── Subscriber ────────────────────────────────────────
        self.create_subscription(String, '/obj_detect', self._obj_callback, 10)
        self.create_subscription(String, '/head/state', self._head_state_callback, 10)

        # ── PID setup ─────────────────────────────────────────
        self.pid = [PIDControl() for _ in range(2)]

        # TILT — tambahkan setTime() sesuai referensi ROS1
        self.pid[HEAD_TILT].setConstant(Kp=TILT_KP, Ki=TILT_KI, Kd=TILT_KD)
        self.pid[HEAD_TILT].setTime(Ti=TILT_TI, Td=TILT_TD)
        self.pid[HEAD_TILT].setRange(InMin=0, InMax=FRAME_H, OutMin=TILT_MIN, OutMax=TILT_MAX)
        self.pid[HEAD_TILT].setSetPoints(TILT_SP)

        # PAN — tambahkan setTime() sesuai referensi ROS1
        self.pid[HEAD_PAN].setConstant(Kp=PAN_KP, Ki=PAN_KI, Kd=PAN_KD)
        self.pid[HEAD_PAN].setTime(Ti=PAN_TI, Td=PAN_TD)
        self.pid[HEAD_PAN].setRange(InMin=0, InMax=FRAME_W, OutMin=PAN_MIN, OutMax=PAN_MAX)
        self.pid[HEAD_PAN].setSetPoints(PAN_SP)

        for i in range(2):
            self.pid[i].Init()
            self.pid[i].setError(0)
            self.pid[i].setEnableWindUpLimit()
            self.pid[i].setEnableWindUpCrossing()

        # ── EKF ───────────────────────────────────────────────
        self.ekf = BallEKF(
            dt=0.05,
            process_noise_pos=2.0,
            process_noise_vel=8.0,
            measure_noise=6.0
        )

        # ── State ─────────────────────────────────────────────
        self.state          = 0       # 0=idle, 2=active
        self.head_enabled   = False
        self.head_mode      = "scan"

        # Posisi servo kepala saat ini
        self.pan            = 0.0
        self.tilt           = -0.3

        # Scan
        self.scan_dir       = 1
        self.last_detect_time = self.get_clock().now()

        # Loop timing
        self._last_loop_time = time.time()

        # ── Timer loop 20 Hz ──────────────────────────────────
        self.timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info("Head Tracking + Auto Scan + EKF Initialized")

    # ── CALLBACKS ─────────────────────────────────────────────────────────────

    def _head_state_callback(self, msg: String):
        """Aktifkan/nonaktifkan head tracking dari topic /head/state"""
        if msg.data == "scan":
            self.state = 2
            self._enter_scan_mode()
            self.get_logger().info("[HeadControl] State → SCAN (ACTIVE)")
        elif msg.data == "off":
            self.state = 0
            self.head_enabled = False
            self.get_logger().info("[HeadControl] State → OFF")

    def _obj_callback(self, msg: String):
        """
        Terima posisi bola dari /obj_detect → format 'cx,cy'.
        Update EKF lalu hitung PID.
        """
        data = msg.data.strip()
        if not data:
            return

        for line in data.split('\n'):
            parts = line.strip().split(',')
            if len(parts) < 2:
                continue
            try:
                cx = float(parts[0])
                cy = float(parts[1])
            except ValueError:
                continue

            if cx <= 0 or cy <= 0:
                continue

            # Update waktu deteksi terakhir
            self.last_detect_time = self.get_clock().now()

            # Update EKF dengan pengukuran baru
            self.ekf.update(px_meas=cx, py_meas=cy)
            px_ekf, py_ekf = self.ekf.get_position()

            # Publish data mentah dan EKF untuk debug
            raw_msg = Vector3()
            raw_msg.x, raw_msg.y = cx, cy
            self.ball_meas_pub.publish(raw_msg)

            ekf_msg = Vector3()
            ekf_msg.x, ekf_msg.y = px_ekf, py_ekf
            self.ball_ekf_pub.publish(ekf_msg)

            # Hitung PID dari posisi EKF
            error_x = px_ekf - PAN_SP
            error_y = py_ekf - TILT_SP

            if abs(error_x) > DEADBAND_X:
                self.pid[HEAD_PAN].calculate(px_ekf)
                self.pan += self.pid[HEAD_PAN].getOutput()

            if abs(error_y) > DEADBAND_Y:
                self.pid[HEAD_TILT].calculate(py_ekf)
                self.tilt += self.pid[HEAD_TILT].getOutput()

            # Clamp ke joint limit
            self.pan  = max(-PAN_LIMIT,  min(PAN_LIMIT,  self.pan))
            self.tilt = max(TILT_LOWER,  min(TILT_UPPER, self.tilt))

    # ── CONTROL LOOP ──────────────────────────────────────────────────────────

    def _control_loop(self):
        """Loop 20 Hz: prediksi EKF + kirim perintah servo kepala"""
        if self.state != 2:
            return

        # Aktifkan head_control_module jika belum
        if not self.head_enabled:
            self._enable_head_module()
            self.head_enabled = True

        # Hitung dt untuk prediksi EKF
        now_wall = time.time()
        dt_loop  = max(min(now_wall - self._last_loop_time, 0.2), 1e-4)
        self._last_loop_time = now_wall

        # Hitung waktu sejak deteksi terakhir
        dt_detect = (self.get_clock().now() - self.last_detect_time).nanoseconds / 1e9

        if dt_detect > SCAN_TIMEOUT:
            # Bola hilang → scan mode
            if self.head_mode != "scan":
                self._enter_scan_mode()
            self._scan_head()
        else:
            # Bola ada → track mode
            if self.head_mode != "track":
                self._enter_track_mode()

            # Prediksi EKF walaupun tidak ada deteksi baru
            self.ekf.predict(dt=dt_loop)
            if self.ekf.should_reset():
                self.ekf.reset()

            # Publish posisi servo (HeadJoint)
            self._publish_head_joint(self.pan, self.tilt)

    # ── MODE HELPER ───────────────────────────────────────────────────────────

    def _enter_scan_mode(self):
        self.head_mode = "scan"
        self.ekf.reset()
        for i in range(2):
            self.pid[i].reset()
        self.pan  = 0.0
        self.tilt = -0.3

    def _enter_track_mode(self):
        self.head_mode = "track"

    def _scan_head(self):
        """Sweep kepala kiri-kanan saat bola tidak terdeteksi"""
        self.pan += SCAN_STEP * self.scan_dir
        if self.pan >= PAN_LIMIT:
            self.pan      = PAN_LIMIT
            self.scan_dir = -1
        elif self.pan <= -PAN_LIMIT:
            self.pan      = -PAN_LIMIT
            self.scan_dir = 1
        self._publish_head_joint(self.pan, SCAN_TILT)

    # ── PUBLISH ───────────────────────────────────────────────────────────────

    def _publish_head_joint(self, pan: float, tilt: float):
        """
        Publish perintah posisi servo kepala ke /robotis/head_control/set_joint_states.
        Setara dengan motion.HeadJoint() di ROS1.
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name         = ['head_pan', 'head_tilt']
        msg.position     = [float(pan), float(tilt)]
        self.head_pub.publish(msg)

    def _enable_head_module(self):
        """Aktifkan head_control_module di robotis_controller"""
        msg      = String()
        msg.data = "head_control_module"
        self.module_pub.publish(msg)
        self.get_logger().info("[HeadControl] head_control_module enabled")


def main():
    rclpy.init()
    node = HeadControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[HeadControl] Node dihentikan.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
