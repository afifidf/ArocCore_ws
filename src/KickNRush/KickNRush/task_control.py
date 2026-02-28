# task_control.py — State machine utama robot OP3 AROC26
# Alur: IDLE → SEARCH → APPROACH → ORBIT → ADJUST → KICK
# Kapan saja: jika jatuh → GET_UP → kembali ke state sebelumnya

import time
from enum import Enum, auto

from std_msgs.msg import String, Int32
from sensor_msgs.msg import Imu
from op3_walking_module_msgs.msg import WalkingParam

from .motion_orbit import MotionOrbit
from .motion_crab import MotionCrab
from .goal_alignment import GoalAlignment

# Konstanta frame — harus sama dengan vision.py
FRAME_W = 640
FRAME_H = 480


class State(Enum):
    IDLE     = auto()   # diam, tunggu tombol start
    SEARCH   = auto()   # cari bola
    APPROACH = auto()   # maju ke bola
    ORBIT    = auto()   # orbit sampai sejajar gawang
    ADJUST   = auto()   # crab fine-tune posisi kaki
    KICK     = auto()   # tendang
    GET_UP   = auto()   # bangun setelah jatuh


# Tuning parameter
BALL_WIDTH_FAR       = 50     # px — bola masih jauh
BALL_WIDTH_CLOSE     = 100    # px — bola cukup dekat, mulai orbit
BALL_CENTER_DEADBAND = 40     # px — toleransi posisi bola dari tengah frame
APPROACH_X           = 0.030  # m  — kecepatan maju approach
APPROACH_PERIOD      = 0.55   # s
SEARCH_ANGLE         = 10.0   # deg — rotasi saat search
SEARCH_PERIOD        = 0.55   # s
BALL_LOST_TIMEOUT    = 1.5    # s  — timeout bola hilang sebelum kembali SEARCH
FALL_THRESHOLD       = 7.0    # m/s² — ambang akselerasi untuk fall detection

# Nomor page action
ACTION_KICK_RIGHT    = 83
ACTION_KICK_LEFT     = 84
ACTION_GETUP_FRONT   = 122
ACTION_GETUP_BACK    = 123


class TaskControl:
    """
    State machine utama. Dipanggil dari main.py setiap 20 Hz via update().
    Mengorkestrasi MotionOrbit, MotionCrab, GoalAlignment, dan publisher walking/action.
    """

    def __init__(self, node):
        self.node        = node
        self.state       = State.IDLE
        self._prev_state = State.IDLE  # untuk recovery setelah GET_UP

        # Inisialisasi modul
        self.orbit     = MotionOrbit(node)
        self.crab      = MotionCrab(node)
        self.alignment = GoalAlignment(node)

        # Publisher
        self._pub_walking_cmd   = node.create_publisher(String, '/robotis/walking/command', 10)
        self._pub_walking_param = node.create_publisher(WalkingParam, '/robotis/walking/set_params', 10)
        self._pub_action        = node.create_publisher(Int32, '/robotis/action/page_num', 10)
        self._pub_module        = node.create_publisher(String, '/robotis/enable_ctrl_module', 10)

        # Subscriber
        node.create_subscription(String, '/obj_detect', self._ball_callback, 10)
        node.create_subscription(String, '/obj_detect_ball_bbox', self._ball_bbox_callback, 10)
        node.create_subscription(String, '/robotis/open_cr/button', self._button_callback, 10)
        node.create_subscription(Imu, '/robotis/open_cr/imu', self._imu_callback, 10)

        # Data internal
        self._ball_cx        = None   # posisi x bola di frame (px)
        self._ball_cy        = None   # posisi y bola di frame (px)
        self._ball_width     = None   # lebar bbox bola (px) — estimasi jarak
        self._ball_detected  = False
        self._last_ball_time = 0.0
        self._imu_accel_x    = 0.0   # untuk fall detection
        self._task_timer     = time.time()

        self.node.get_logger().info("[TaskControl] Init — state: IDLE")

    # ── CALLBACKS ─────────────────────────────────────────────────────────────

    def _ball_callback(self, msg: String):
        """Terima posisi bola dari /obj_detect → 'cx,cy'"""
        data = msg.data.strip()
        if not data:
            return
        try:
            parts = data.split(',')
            if len(parts) >= 2:
                self._ball_cx        = float(parts[0])
                self._ball_cy        = float(parts[1])
                self._ball_detected  = True
                self._last_ball_time = time.time()
        except ValueError:
            pass

    def _ball_bbox_callback(self, msg: String):
        """Terima bbox bola dari /obj_detect_ball_bbox → 'cx,cy,w,h'"""
        data = msg.data.strip()
        if not data:
            return
        try:
            parts = data.split(',')
            if len(parts) >= 3:
                self._ball_cx        = float(parts[0])
                self._ball_cy        = float(parts[1])
                self._ball_width     = float(parts[2])
                self._ball_detected  = True
                self._last_ball_time = time.time()
        except ValueError:
            pass

    def _button_callback(self, msg: String):
        """Handler tombol OpenCR: 'start' mulai, 'mode' stop ke IDLE"""
        btn = msg.data.strip()
        if btn == "start" and self.state == State.IDLE:
            self.node.get_logger().info("[TaskControl] START → SEARCH")
            self._enable_walking_module()
            self.alignment.reset_yaw()
            self._set_state(State.SEARCH)
        elif btn == "mode":
            self.node.get_logger().info("[TaskControl] MODE → IDLE")
            self._stop_walking()
            self._set_state(State.IDLE)

    def _imu_callback(self, msg: Imu):
        """Simpan akselerasi x untuk fall detection"""
        self._imu_accel_x = msg.linear_acceleration.x

    # ── MAIN LOOP ─────────────────────────────────────────────────────────────

    def update(self):
        """Dipanggil setiap 20 Hz dari main.py"""
        # Fall detection di semua state kecuali IDLE dan GET_UP
        if self.state not in (State.IDLE, State.GET_UP):
            if self._is_falling():
                self._prev_state = self.state
                self._set_state(State.GET_UP)
                return

        ball_lost = (time.time() - self._last_ball_time) > BALL_LOST_TIMEOUT

        if   self.state == State.IDLE:     self._state_idle()
        elif self.state == State.SEARCH:   self._state_search(ball_lost)
        elif self.state == State.APPROACH: self._state_approach(ball_lost)
        elif self.state == State.ORBIT:    self._state_orbit(ball_lost)
        elif self.state == State.ADJUST:   self._state_adjust(ball_lost)
        elif self.state == State.KICK:     self._state_kick()
        elif self.state == State.GET_UP:   self._state_get_up()

    # ── STATE HANDLERS ────────────────────────────────────────────────────────

    def _state_idle(self):
        pass  # handling di _button_callback

    def _state_search(self, ball_lost: bool):
        """Putar badan cari bola. Head tracking scan otomatis dari headControl26."""
        if self._ball_detected and not ball_lost:
            self.node.get_logger().info("[TaskControl] Bola ketemu → APPROACH")
            self._set_state(State.APPROACH)
            return
        param = WalkingParam()
        param.x_move_amplitude     = 0.0
        param.y_move_amplitude     = 0.0
        param.angle_move_amplitude = SEARCH_ANGLE * 3.14159 / 180.0  # deg → rad
        param.period_time          = SEARCH_PERIOD
        self._pub_walking_param.publish(param)
        self._walking_start()

    def _state_approach(self, ball_lost: bool):
        """Maju ke bola. Estimasi jarak dari lebar bbox atau posisi cy."""
        if ball_lost:
            self.node.get_logger().info("[TaskControl] Bola hilang saat APPROACH → SEARCH")
            self._stop_walking()
            self._set_state(State.SEARCH)
            return

        # Estimasi jarak: pakai ball_width jika ada, else pakai cy
        dist = (self._ball_width if self._ball_width is not None
                else (FRAME_H - self._ball_cy) if self._ball_cy is not None else 0)

        if dist >= BALL_WIDTH_CLOSE:
            self.node.get_logger().info(f"[TaskControl] Bola dekat (est={dist:.0f}) → ORBIT")
            self._stop_walking()
            self._set_state(State.ORBIT)
            return

        # Pastikan walking_module tetap aktif
        self._enable_walking_module_soft()

        param = WalkingParam()
        param.x_move_amplitude     = APPROACH_X
        param.y_move_amplitude     = 0.0
        param.angle_move_amplitude = 0.0
        param.period_time          = APPROACH_PERIOD
        self._pub_walking_param.publish(param)
        self._walking_start()

    def _state_orbit(self, ball_lost: bool):
        """Orbit mengelilingi bola. Arah otomatis dari GoalAlignment (IMU+YOLO)."""
        if ball_lost:
            self.node.get_logger().info("[TaskControl] Bola hilang saat ORBIT → SEARCH")
            self._stop_walking()
            self._set_state(State.SEARCH)
            return

        if self.alignment.is_aligned():
            self.node.get_logger().info("[TaskControl] Sejajar gawang → ADJUST")
            self._stop_walking()
            self._set_state(State.ADJUST)
            return

        # Pastikan walking_module tetap aktif (antisipasi bentrok dengan head_control_module)
        self._enable_walking_module_soft()

        direction = self.alignment.get_orbit_direction()
        self.orbit.set_direction(direction)
        x, y, angle_deg, period = self.orbit.get_params()

        param = WalkingParam()
        param.x_move_amplitude     = x
        param.y_move_amplitude     = y
        param.angle_move_amplitude = angle_deg * 3.14159 / 180.0  # deg → rad
        param.period_time          = period
        self._pub_walking_param.publish(param)
        self._walking_start()

    def _state_adjust(self, ball_lost: bool):
        """Crab walk slow untuk posisikan bola di tengah frame sebelum tendang."""
        if ball_lost:
            self.node.get_logger().info("[TaskControl] Bola hilang saat ADJUST → SEARCH")
            self._stop_walking()
            self._set_state(State.SEARCH)
            return

        if self._ball_cx is None:
            return

        direction = self.crab.estimate_direction_from_ball(
            self._ball_cx, frame_w=640.0, deadband=BALL_CENTER_DEADBAND)

        if direction is None:
            self.node.get_logger().info("[TaskControl] Bola di tengah → KICK")
            self._stop_walking()
            self._set_state(State.KICK)
            return

        self.crab.set_direction(direction, speed_mode="slow")
        x, y, angle, period = self.crab.get_params()

        # Pastikan walking_module tetap aktif
        self._enable_walking_module_soft()

        param = WalkingParam()
        param.x_move_amplitude     = x
        param.y_move_amplitude     = y
        param.angle_move_amplitude = 0.0  # crab tidak rotasi
        param.period_time          = period
        self._pub_walking_param.publish(param)
        self._walking_start()

    def _state_kick(self):
        """Tendang bola. Kaki kiri/kanan dipilih dari posisi bola di frame."""
        self._enable_action_module()
        time.sleep(0.3)

        if self._ball_cx is not None and self._ball_cx < 320:
            self.node.get_logger().info("[TaskControl] TENDANG KAKI KIRI!")
            self._do_action(ACTION_KICK_LEFT)
        else:
            self.node.get_logger().info("[TaskControl] TENDANG KAKI KANAN!")
            self._do_action(ACTION_KICK_RIGHT)

        time.sleep(3.0)  # tunggu animasi tendang selesai
        self._enable_walking_module()
        self.alignment.reset_yaw()
        self._set_state(State.SEARCH)

    def _state_get_up(self):
        """Bangun setelah jatuh. Arah jatuh dideteksi dari akselerasi IMU."""
        self.node.get_logger().info(f"[TaskControl] GET UP! accel_x={self._imu_accel_x:.2f}")
        self._stop_walking()
        self._enable_action_module()
        time.sleep(0.3)

        if self._imu_accel_x > FALL_THRESHOLD:
            self.node.get_logger().info("[TaskControl] Jatuh BELAKANG → GetUpBack")
            self._do_action(ACTION_GETUP_BACK)
        else:
            self.node.get_logger().info("[TaskControl] Jatuh DEPAN → GetUpFront")
            self._do_action(ACTION_GETUP_FRONT)

        time.sleep(4.0)  # tunggu animasi bangun selesai
        self._enable_walking_module()
        self.alignment.reset_yaw()
        self.node.get_logger().info(
            f"[TaskControl] Bangun! Lanjut: {self._prev_state.name}")
        self._set_state(self._prev_state)

    # ── UTILITY ───────────────────────────────────────────────────────────────

    def _set_state(self, new_state: State):
        self.node.get_logger().info(f"[TaskControl] {self.state.name} → {new_state.name}")
        self.state       = new_state
        self._task_timer = time.time()

    def _is_falling(self) -> bool:
        return abs(self._imu_accel_x) > FALL_THRESHOLD

    def _walking_start(self):
        msg = String(); msg.data = 'start'
        self._pub_walking_cmd.publish(msg)

    def _stop_walking(self):
        msg = String(); msg.data = 'stop'
        self._pub_walking_cmd.publish(msg)

    def _do_action(self, page_num: int):
        msg = Int32(); msg.data = page_num
        self._pub_action.publish(msg)

    def _enable_walking_module(self):
        msg = String(); msg.data = 'walking_module'
        self._pub_module.publish(msg)
        time.sleep(0.5)

    def _enable_walking_module_soft(self):
        """Publish walking_module tanpa sleep — aman dipanggil tiap loop 20Hz.
        Dipakai di state ORBIT/APPROACH/ADJUST agar walking_module tidak
        di-override oleh head_control_module dari headControl26."""
        now = time.time()
        if not hasattr(self, '_last_module_pub') or (now - self._last_module_pub) > 1.0:
            msg = String(); msg.data = 'walking_module'
            self._pub_module.publish(msg)
            self._last_module_pub = now

    def _enable_action_module(self):
        msg = String(); msg.data = 'action_module'
        self._pub_module.publish(msg)
        time.sleep(0.3)
