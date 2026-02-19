# loc: aroc26/src/headControl26/headControl26/mainHeadControl.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from sensor_msgs.msg import JointState
from .motionPID import PIDControl
from .ballEKF import BallEKF
from geometry_msgs.msg import Vector3
import time

HEAD_PAN = 0
HEAD_TILT = 1


class ButtonStandScan(Node):

    def __init__(self):
        super().__init__('button_stand_scan_node')

        # ==============================
        # Subscriber
        # ==============================
        self.create_subscription(
            String,
            '/robotis/open_cr/button',
            self.button_callback,
            10
        )

        self.create_subscription(
            String,
            '/obj_detect',
            self.obj_callback,
            10
        )

        # ==============================
        # Publisher
        # ==============================
        self.module_pub = self.create_publisher(
            String, '/robotis/enable_ctrl_module', 10
        )

        self.page_pub = self.create_publisher(
            Int32, '/robotis/action/page_num', 10
        )

        self.head_pub = self.create_publisher(
            JointState,
            '/robotis/head_control/set_joint_states',
            10
        )

        # Topic buat ngukur bola
        self.ball_meas_pub = self.create_publisher(
            Vector3,
            '/vision/ball_measurement',
            10
        )

        # ============================================================
        # [EKF] Publisher untuk state EKF (posisi + kecepatan)
        # Berguna untuk debugging dan monitoring performa EKF
        # Format Vector3: x=px_filtered, y=py_filtered, z=0
        # ============================================================
        self.ball_ekf_pub = self.create_publisher(
            Vector3,
            '/vision/ball_ekf',
            10
        )

        # ==============================
        # STATE
        # ==============================
        self.state = 0
        self.head_enabled = False
        self.head_mode = "scan"   # scan / track

        # ==============================
        # PID SETUP
        # ==============================
        self.pid = [PIDControl() for _ in range(2)]

        # TILT
        self.pid[HEAD_TILT].setConstant(Kp=1.0, Ki=0.0, Kd=0.2)
        self.pid[HEAD_TILT].setRange(0, 480, -1.2, 0.0)
        self.pid[HEAD_TILT].setSetPoints(240)

        # PAN
        self.pid[HEAD_PAN].setConstant(Kp=1.4, Ki=0.0, Kd=0.25)  # Kp=1.0,Ki=0.0, Kd=0.2 ; Kp=1.35,Ki=0.0, Kd=0.3
        self.pid[HEAD_PAN].setRange(0, 640, -1.2, 1.2)
        self.pid[HEAD_PAN].setSetPoints(320)

        for i in range(2):
            self.pid[i].Init()
            self.pid[i].setEnableWindUpLimit()
            self.pid[i].setEnableWindUpCrossing()

        # ============================================================
        # [EKF] Inisialisasi Extended Kalman Filter
        #
        # Parameter tuning:
        #   dt              = 0.05 detik (sesuai timer 20 Hz)
        #   process_noise_pos = 2.0  → agak responsif terhadap gerakan bola
        #   process_noise_vel = 8.0  → estimasi kecepatan cukup agresif
        #   measure_noise     = 6.0  → mempercayai measurement cukup besar
        #                              (lebih kecil = lebih presisi tapi lebih berisik)
        #
        # Cara tuning:
        #   - Jika tracking terlalu lambat merespons → perkecil measure_noise
        #   - Jika tracking terlalu berisik/jitter   → perbesar measure_noise
        #   - Jika prediksi saat bola hilang kurang akurat → perbesar process_noise_vel
        # ============================================================
        self.ekf = BallEKF(
            dt=0.05,
            process_noise_pos=2.0,
            process_noise_vel=8.0,
            measure_noise=6.0
        )

        # ============================================================
        # [EKF] Simpan waktu terakhir control_loop untuk hitung dt aktual
        # Digunakan pada langkah predict EKF di control_loop
        # ============================================================
        self._last_loop_time = time.time()

        # ==============================
        # SCAN PARAMETER
        # ==============================
        self.pan = 0.0
        self.tilt = -0.3
        self.scan_dir = 1

        self.last_detection_time = self.get_clock().now()
        self.scan_timeout = 0.7  # 1.0

        # ==============================
        # MAIN LOOP
        # ==============================
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Head Tracking + Auto Scan + EKF Initialized")

    # ==========================================================
    # BUTTON CALLBACK
    # ==========================================================
    def button_callback(self, msg):

        if msg.data == "user":
            self.enable_action_module()
            self.play_page(15)

        elif msg.data == "start":
            self.enable_action_module()
            self.play_page(15)
            self.state = 2
            self.get_logger().info("Stand sequence triggered")

    # ==========================================================
    # CONTROL LOOP (20 Hz)
    # ==========================================================
    def control_loop(self):

        if self.state != 2:
            return

        if not self.head_enabled:
            self.enable_head_module()
            self.head_enabled = True

        now = self.get_clock().now()
        dt = (now - self.last_detection_time).nanoseconds / 1e9

        # ============================================================
        # [EKF] Hitung dt aktual untuk langkah predict
        # dt aktual digunakan agar prediksi EKF akurat dengan waktu nyata
        # ============================================================
        now_wall = time.time()
        dt_loop = now_wall - self._last_loop_time
        dt_loop = max(dt_loop, 1e-4)   # hindari dt nol
        dt_loop = min(dt_loop, 0.2)    # clamp maksimal 200ms
        self._last_loop_time = now_wall

        if dt > self.scan_timeout:
            if self.head_mode != "scan":
                self.enter_scan_mode()
            self.scan_head()
        else:
            if self.head_mode != "track":
                self.enter_track_mode()

            # ============================================================
            # [EKF] Langkah PREDICT di setiap loop control (20 Hz)
            # Walaupun tidak ada measurement baru dari YOLO, EKF tetap
            # memprediksi posisi bola berdasarkan kecepatan yang diestimasi.
            # Ini membuat PID mendapat input yang lebih smooth dan kontinyu.
            #
            # [FIX] DOUBLE PREDICT DIHINDARI dengan cara:
            # - predict() di sini hanya dipanggil di control_loop (20 Hz)
            # - update() di obj_callback TIDAK memanggil predict internal lagi
            #   (di ballEKF, update() sudah dipisah: hanya koreksi, predict
            #    dilakukan di sini di control_loop)
            # Sebelumnya: update() di ballEKF memanggil F@x dan F@P@F.T sendiri
            # DAN control_loop juga memanggil predict() → state dipropagasi 2x.
            # Sekarang: predict() hanya ada di satu tempat (control_loop).
            # ============================================================
            self.ekf.predict(dt=dt_loop)

            # ============================================================
            # [EKF] Cek apakah EKF perlu di-reset (bola hilang terlalu lama)
            # ============================================================
            if self.ekf.should_reset():
                self.ekf.reset()
                self.get_logger().info("[EKF] Filter di-reset karena bola hilang terlalu lama")

            self.track_head()

    # ==========================================================
    # OBJECT CALLBACK
    # ==========================================================
    def obj_callback(self, msg):

        # ============================================================
        # [FIX] Guard: jangan proses deteksi jika robot belum dalam
        # state aktif (state == 2). Sebelumnya EKF dan PID ikut
        # berjalan walau robot belum berdiri → state pan/tilt kotor
        # saat tracking pertama kali dimulai.
        #
        # [FIX v2] EKF update + publish debug tetap bisa berjalan
        # walau state != 2, agar /vision/ball_ekf bisa dimonitor
        # kapan saja untuk keperluan debugging dan tuning parameter.
        # Yang TIDAK boleh berjalan saat state != 2 adalah:
        #   - PID calculate (mengubah self.pan / self.tilt)
        #   - publish servo ke robot
        # ============================================================
        pid_active = (self.state == 2)

        lines = msg.data.strip().split('\n')

        for line in lines:
            parts = line.strip().split(',')

            if len(parts) == 2:
                try:
                    cx = int(parts[0])
                    cy = int(parts[1])

                    center_x = 640 // 2
                    center_y = 480 // 2

                    deadband = 8  # pixel toleransi

                    if cx > 0 and cy > 0:
                        self.last_detection_time = self.get_clock().now()

                        # PUBLISH KE EKF (raw measurement untuk monitoring)
                        meas = Vector3()
                        meas.x = float(cx - center_x)
                        meas.y = float(cy - center_y)
                        meas.z = 0.0
                        self.ball_meas_pub.publish(meas)

                        # ============================================================
                        # [EKF] UPDATE EKF dengan measurement baru dari YOLO
                        # EKF menggabungkan prediksi model dengan measurement sensor
                        # menggunakan Kalman Gain untuk menghasilkan estimasi optimal.
                        # Input  : px, py raw dari YOLO (koordinat pixel absolut)
                        # Output : state terkoreksi [px, py, vx, vy]
                        # ============================================================
                        self.ekf.update(px_meas=float(cx), py_meas=float(cy))

                        # ============================================================
                        # [EKF] Ambil posisi yang sudah difilter EKF
                        # px_ekf, py_ekf sudah smooth dan bebas noise YOLO
                        # ============================================================
                        px_ekf, py_ekf = self.ekf.get_position()

                        # ============================================================
                        # [EKF] Publish posisi EKF untuk monitoring / debugging
                        # ============================================================
                        ekf_msg = Vector3()
                        ekf_msg.x = px_ekf - float(center_x)   # error dari center
                        ekf_msg.y = py_ekf - float(center_y)
                        ekf_msg.z = 0.0
                        self.ball_ekf_pub.publish(ekf_msg)

                        # ============================================================
                        # [EKF] Hitung error menggunakan posisi EKF (bukan raw pixel)
                        # Inilah keuntungan utama EKF:
                        #   - px_ekf/py_ekf lebih smooth → PID lebih stabil
                        #   - Tidak ada spike tiba-tiba dari noise YOLO
                        #   - Gerakan head pan/tilt menjadi halus dan presisi
                        # ============================================================
                        error_x = px_ekf - float(center_x)   # error pan  (+ = bola di kanan)
                        error_y = py_ekf - float(center_y)   # error tilt (+ = bola di bawah)

                        # =========================
                        # DEAD BAND + PID
                        # Hanya aktif jika state == 2 (robot sudah berdiri)
                        # [FIX v2] Guard pid_active memisahkan:
                        #   - EKF update + debug publish → selalu aktif
                        #   - PID + servo publish → hanya saat state == 2
                        # =========================
                        if pid_active:
                            if abs(error_x) >= deadband:
                                # [EKF] PID PAN menggunakan px_ekf (posisi smooth dari EKF)
                                self.pid[HEAD_PAN].setSetPoints(center_x)
                                self.pid[HEAD_PAN].calculate(px_ekf)
                                self.pan += self.pid[HEAD_PAN].getOutput()

                            if abs(error_y) >= deadband:
                                # [EKF] PID TILT menggunakan py_ekf (posisi smooth dari EKF)
                                self.pid[HEAD_TILT].setSetPoints(center_y)
                                self.pid[HEAD_TILT].calculate(py_ekf)
                                self.tilt += self.pid[HEAD_TILT].getOutput()

                            self.pan = max(-1.2, min(1.2, self.pan))
                            self.tilt = max(-1.2, min(0.0, self.tilt))

                        self.get_logger().info(
                            f"[EKF] raw=({cx},{cy}) → ekf=({px_ekf:.1f},{py_ekf:.1f}) "
                            f"| state={'ACTIVE' if pid_active else 'DEBUG-ONLY'} "
                            f"| pan={self.pan:.3f} tilt={self.tilt:.3f}"
                        )

                except Exception:
                    self.get_logger().warn(f"Gagal parse: {line}")

    # ==========================================================
    # MODE TRANSITION
    # ==========================================================
    def enter_scan_mode(self):
        self.head_mode = "scan"

        for i in range(2):
            self.pid[i].Init()

        # ============================================================
        # [EKF] Reset EKF saat masuk scan mode
        # Saat bola hilang dan masuk scan mode, EKF di-reset agar
        # saat bola ditemukan kembali filter mulai fresh (tidak
        # terpengaruh state lama yang mungkin sudah tidak relevan)
        # ============================================================
        self.ekf.reset()

        self.pan = 0.0
        self.tilt = -0.3

        self.get_logger().info("ENTER SCAN MODE — EKF reset")

    def enter_track_mode(self):
        self.head_mode = "track"

        for i in range(2):
            self.pid[i].Init()

        self.get_logger().info("ENTER TRACK MODE")

    # ==========================================================
    # TRACK MODE
    # ==========================================================
    def track_head(self):
        self.publish_servo(self.pan, self.tilt)

    # ==========================================================
    # SCAN MODE
    # ==========================================================
    def scan_head(self):

        self.pan += 0.05 * self.scan_dir

        if self.pan >= 1.2:
            self.pan = 1.2
            self.scan_dir = -1
        elif self.pan <= -1.2:
            self.pan = -1.2
            self.scan_dir = 1

        self.publish_servo(self.pan, -0.4)

    # ==========================================================
    # SERVO PUBLISH
    # ==========================================================
    def publish_servo(self, pan, tilt):

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['head_pan', 'head_tilt']
        msg.position = [pan, tilt]

        self.head_pub.publish(msg)

    # ==========================================================
    # UTIL
    # ==========================================================
    def enable_action_module(self):
        mode = String()
        mode.data = "action_module"
        self.module_pub.publish(mode)

    def enable_head_module(self):
        mode = String()
        mode.data = "head_control_module"
        self.module_pub.publish(mode)
        self.get_logger().info("Head control module enabled")

    def play_page(self, num):
        cmd = Int32()
        cmd.data = num
        self.page_pub.publish(cmd)


def main():
    rclpy.init()
    node = ButtonStandScan()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
