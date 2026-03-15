import time

from std_msgs.msg import String, Int32
from op3_walking_module_msgs.msg import WalkingParam

from motion_module.motion_approach import MotionApproach

# ── Nomor action ──────────────────────────────────────────────
ACTION_STANDUP = 1
ACTION_SIT     = 15

class TaskControl:
    def __init__(self, node):
        self.node  = node
        self.state = 'IDLE'   # IDLE | STANDUP | APPROACH

        # ── Publisher ─────────────────────────────────────────
        self._pub_module  = node.create_publisher(
            String, '/robotis/enable_ctrl_module', 10)
        self._pub_action  = node.create_publisher(
            Int32, '/robotis/action/page_num', 10)
        self._pub_walk_cmd = node.create_publisher(
            String, '/robotis/walking/command', 10)
        self._pub_walk_param = node.create_publisher(
            WalkingParam, '/robotis/walking/set_params', 10)
        self._pub_head = node.create_publisher(
            String, '/head/state', 10)

        # ── Modul approach ────────────────────────────────────
        self.approach = MotionApproach(node)

        # ── Data bola dari headControl26 ──────────────────────
        self._ball_width  = 0.0
        self._last_ball_t = time.time()
        self.BALL_TIMEOUT = 1.5   # s

        # ── State walking ─────────────────────────────────────
        self._is_walking   = False
        self._was_ball_lost = False   # untuk throttle warn ball_lost

        # ── Subscriber bbox bola ──────────────────────────────
        node.create_subscription(
            String,
            '/obj_detect_ball_bbox',
            self._ball_bbox_callback,
            10
        )

        node.get_logger().info("[TaskControl] Init — state: IDLE")

    # ── CALLBACK ──────────────────────────────────────────────

    def _ball_bbox_callback(self, msg: String):
        """
        Terima bbox bola dari vision.py → format 'cx,cy,w,h'.

        Jika string kosong → bola tidak terdeteksi, tidak update _last_ball_t
        sehingga BALL_TIMEOUT akan trigger dan ball_lost = True.
        """
        data = msg.data.strip()
        if not data:
            # String kosong = bola tidak terdeteksi
            # Tidak update _last_ball_t → ball_lost akan trigger setelah BALL_TIMEOUT
            return
        try:
            parts = data.split(',')
            if len(parts) >= 3:
                self._ball_width  = float(parts[2])
                self._last_ball_t = time.time()
        except ValueError:
            pass

    # ── TOMBOL HANDLER ────────────────────────────────────────

    def on_user_pressed(self):
        """
        Toggle IDLE ↔ STANDUP.

        Dipanggil dari buttonHandler saat tombol USER ditekan.
        """
        if self.state == 'IDLE':
            self._log("USER → STANDUP")
            self._enable_action_module()
            self._do_action(ACTION_STANDUP)
            self._head_scan()
            self.state = 'STANDUP'

        else:
            # Dari state apapun (STANDUP / APPROACH) → kembali IDLE
            self._log("USER → SIT → IDLE")
            # Selalu stop walking saat sit, tidak peduli _is_walking
            # (antisipasi kondisi bola hilang tapi walking module masih aktif)
            self._stop_walking()
            self._enable_action_module()
            self._do_action(ACTION_SIT)
            self._head_off()
            self.state       = 'IDLE'
            self._is_walking = False

    def on_start_pressed(self):
        """
        Toggle APPROACH on/off.

        Dipanggil dari buttonHandler saat tombol START ditekan.
        """
        if self.state == 'IDLE':
            self.node.get_logger().warn(
                "START diabaikan — robot belum standup!")
            return

        if self.state == 'STANDUP':
            self._log("START → APPROACH")
            self._enable_walking_module()
            self._is_walking    = False   # reset, akan di-start di update()
            self._was_ball_lost = False   # reset flag bola hilang
            self.state = 'APPROACH'

        elif self.state == 'APPROACH':
            self._log("START → stop paksa → STANDUP")
            self._stop_walking()
            self._is_walking = False
            self.state = 'STANDUP'

    # ── MAIN LOOP (20 Hz) ─────────────────────────────────────

    def update(self):
        """
        Dipanggil timer 20Hz dari buttonHandler.

        Hanya aktif saat state == 'APPROACH'.
        """
        if self.state != 'APPROACH':
            return

        ball_lost = (time.time() - self._last_ball_t) > self.BALL_TIMEOUT

        if ball_lost:
            # Throttle: hanya stop walking & warn SEKALI saat pertama hilang
            # Tidak spam stop/warn setiap 50ms
            if not self._was_ball_lost:
                self.node.get_logger().warn(
                    "[TaskControl] Bola hilang — walking stop, tunggu bola...")
                self._stop_walking()
                self._is_walking    = False
                self._was_ball_lost = True
            return

        # Bola ketemu lagi setelah hilang → reset flag
        if self._was_ball_lost:
            self._log("Bola ketemu lagi → lanjut approach")
            self._was_ball_lost = False

        if self.approach.is_close_enough(self._ball_width):
            self._log("✅ Sudah dekat! Berhenti di depan bola → STANDUP")
            self._stop_walking()
            self._is_walking = False
            self.state = 'STANDUP'
            return

        # Belum dekat → maju
        x, y, angle = self.approach.get_params(self._ball_width)

        param = WalkingParam()
        param.x_move_amplitude     = x
        param.y_move_amplitude     = y
        param.angle_move_amplitude = angle
        # period_time tidak diset → dari param.yaml (650ms)
        self._pub_walk_param.publish(param)

        if not self._is_walking:
            self._start_walking()
            self._is_walking = True

    # ── UTILITY ───────────────────────────────────────────────

    def _enable_action_module(self):
        msg = String(); msg.data = 'action_module'
        self._pub_module.publish(msg)

    def _enable_walking_module(self):
        msg = String(); msg.data = 'walking_module'
        self._pub_module.publish(msg)

    def _do_action(self, page_num: int):
        msg = Int32(); msg.data = page_num
        self._pub_action.publish(msg)
        self._log(f"action page {page_num} dikirim")

    def _start_walking(self):
        msg = String(); msg.data = 'start'
        self._pub_walk_cmd.publish(msg)

    def _stop_walking(self):
        msg = String(); msg.data = 'stop'
        self._pub_walk_cmd.publish(msg)

    def _head_scan(self):
        msg = String(); msg.data = 'scan'
        self._pub_head.publish(msg)
        self._log("head → SCAN")

    def _head_off(self):
        msg = String(); msg.data = 'off'
        self._pub_head.publish(msg)
        self._log("head → OFF")

    def _log(self, text: str):
        self.node.get_logger().info(f"[TaskControl/{self.state}] {text}")
