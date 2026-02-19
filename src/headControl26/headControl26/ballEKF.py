# loc: aroc26/src/headControl26/headControl26/ballEKF.py
#
# ============================================================
# BallEKF — Extended Kalman Filter untuk tracking posisi bola
# ============================================================
# Tujuan:
#   Memperhalus (smooth) dan mempercepat respons head tracking
#   dengan cara memfilter noise dari deteksi YOLO dan
#   memprediksi posisi bola di antara frame deteksi.
import numpy as np
import time


class BallEKF:
    """
    Extended Kalman Filter untuk estimasi posisi dan kecepatan bola.

    Digunakan untuk:
    - Meredam noise dari deteksi YOLO (smoothing)
    - Memprediksi posisi bola saat tidak ada deteksi
    - Menghasilkan input PID yang lebih stabil dan presisi

    State   : [px, py, vx, vy]
    Measure : [px, py]
    """

    def __init__(self,
                 dt=0.05,
                 process_noise_pos=1.0,
                 process_noise_vel=5.0,
                 measure_noise=8.0):
        
        self.x = np.array([320.0, 240.0, 0.0, 0.0])  # state awal
        self.P = np.eye(4) * 500.0  # ketidakpastian awal tinggi
        self.Q = np.diag([
            process_noise_pos,   # noise posisi x
            process_noise_pos,   # noise posisi y
            process_noise_vel,   # noise kecepatan x
            process_noise_vel    # noise kecepatan y
        ])

        # ============================================================
        # [EKF] MATRIKS NOISE PENGUKURAN R
        # Mewakili noise dari sensor (YOLO pixel detection)
        # R besar → filter kurang percaya measurement, lebih smooth
        # R kecil → filter mengikuti measurement lebih ketat
        # ============================================================
        self.R = np.eye(2) * measure_noise  # noise pengukuran [px, py]

        # ============================================================
        # [EKF] MATRIKS OBSERVASI H
        # Menghubungkan state [px, py, vx, vy] ke measurement [px, py]
        # Hanya posisi yang diukur, bukan kecepatan
        # ============================================================
        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],  # px dari state
            [0.0, 1.0, 0.0, 0.0],  # py dari state
        ])

        # ============================================================
        # [EKF] PARAMETER WAKTU
        # ============================================================
        self.dt_default = dt          # dt default jika tidak ada info waktu
        self.last_time = None         # waktu terakhir update (untuk hitung dt aktual)

        # ============================================================
        # [EKF] FLAG INISIALISASI
        # Filter baru aktif setelah menerima measurement pertama
        # ============================================================
        self.initialized = False      # False = belum ada measurement pertama

        # ============================================================
        # [EKF] PARAMETER PREDICT-ONLY (saat bola hilang)
        # Batas waktu prediksi tanpa measurement sebelum filter di-reset
        # ============================================================
        self.max_predict_time = 1.0   # detik — jika > ini, reset filter
        self.last_update_time = None  # waktu terakhir menerima measurement

    # ================================================================
    # [EKF] BUILD F — Matriks Transisi State
    # F bergantung pada dt (waktu antar frame)
    # ================================================================
    def _build_F(self, dt):
        """
        Buat matriks transisi state F berdasarkan dt.

        Model: px += vx*dt, py += vy*dt, vx konstan, vy konstan

        F = [[1, 0, dt,  0],
             [0, 1,  0, dt],
             [0, 0,  1,  0],
             [0, 0,  0,  1]]
        """
        F = np.array([
            [1.0, 0.0,  dt, 0.0],
            [0.0, 1.0, 0.0,  dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
        return F

    # ================================================================
    # [EKF] PREDICT — Langkah Prediksi
    # Dipanggil setiap loop control untuk maju satu step waktu
    # ================================================================
    def predict(self, dt=None):
        """
        Langkah prediksi EKF: estimasi state di waktu berikutnya
        berdasarkan model dinamika (constant velocity).

        Args:
            dt: selang waktu (detik). Jika None, pakai dt_default.

        Returns:
            x_pred: state yang diprediksi [px, py, vx, vy]
        """
        if not self.initialized:
            # Belum ada measurement pertama, tidak perlu predict
            return self.x

        if dt is None:
            dt = self.dt_default

        # Clamp dt untuk mencegah prediksi meledak jika ada jeda panjang
        dt = min(dt, 0.2)  # maksimal 200ms per step

        # [EKF] Bangun matriks transisi F untuk dt saat ini
        F = self._build_F(dt)

        # [EKF] PREDICT STEP
        # x_pred = F * x  (propagasi state)
        self.x = F @ self.x

        # [EKF] Propagasi kovarians
        # P_pred = F * P * F^T + Q
        self.P = F @ self.P @ F.T + self.Q

        # Clamp posisi agar tidak keluar dari resolusi kamera
        self.x[0] = np.clip(self.x[0], 0.0, 640.0)  # px
        self.x[1] = np.clip(self.x[1], 0.0, 480.0)  # py

        return self.x

    # ================================================================
    # [EKF] UPDATE — Langkah Koreksi dengan Measurement
    # Dipanggil saat ada deteksi bola dari YOLO (di obj_callback).
    #
    # [FIX] DOUBLE PREDICT:
    # Sebelumnya update() memanggil predict internal (F@x, F@P@F.T+Q)
    # DAN control_loop juga memanggil predict() → state dipropagasi 2x
    # per measurement, menghasilkan estimasi yang terlalu jauh melompat.
    # Sekarang: update() HANYA melakukan koreksi Kalman (tidak predict).
    # Predict dilakukan HANYA di control_loop melalui predict().
    # ================================================================
    def update(self, px_meas, py_meas):
        """
        Langkah update EKF: HANYA koreksi estimasi menggunakan measurement.
        TIDAK ada predict di dalam sini — predict dilakukan di control_loop.

        Args:
            px_meas: posisi x bola dalam pixel (dari YOLO)
            py_meas: posisi y bola dalam pixel (dari YOLO)

        Returns:
            x_updated: state setelah dikoreksi [px, py, vx, vy]
        """
        now = time.time()

        if not self.initialized:
            # ============================================================
            # [EKF] INISIALISASI PERTAMA
            # Set state awal dari measurement pertama.
            # last_time diisi agar predict() berikutnya punya referensi dt.
            # ============================================================
            self.x[0] = float(px_meas)
            self.x[1] = float(py_meas)
            self.x[2] = 0.0   # kecepatan awal = 0 (belum ada referensi)
            self.x[3] = 0.0
            self.P = np.eye(4) * 500.0   # reset kovarians awal
            self.initialized = True
            self.last_time = now
            self.last_update_time = now
            return self.x

        # ============================================================
        # [EKF] UPDATE STEP SAJA (Kalman Gain + Koreksi)
        # State sudah dipropagasi oleh predict() di control_loop.
        # Di sini hanya koreksi berdasarkan measurement baru.
        # ============================================================
        z = np.array([float(px_meas), float(py_meas)])  # measurement vector

        # Innovation / residual: selisih pengukuran vs prediksi state saat ini
        y_innov = z - self.H @ self.x

        # Innovation covariance: S = H * P * H^T + R
        S = self.H @ self.P @ self.H.T + self.R

        # Kalman Gain: K = P * H^T * S^-1
        # K mengatur seberapa besar measurement dipercaya vs prediksi
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # Koreksi state: x = x_pred + K * y_innov
        self.x = self.x + K @ y_innov

        # ============================================================
        # [FIX] JOSEPH FORM untuk update kovarians P
        # Sebelumnya: P = (I - K*H) * P  → standard form
        # Standard form tidak numerically stable karena bisa menghasilkan
        # P tidak simetris / tidak positif definit akibat floating point error.
        # Joseph form: P = (I-KH)*P*(I-KH)^T + K*R*K^T
        # Menjamin P tetap simetris dan positif semi-definit.
        # ============================================================
        I = np.eye(4)
        IKH = I - K @ self.H
        self.P = IKH @ self.P @ IKH.T + K @ self.R @ K.T  # [FIX] Joseph form

        # Clamp posisi hasil update
        self.x[0] = np.clip(self.x[0], 0.0, 640.0)
        self.x[1] = np.clip(self.x[1], 0.0, 480.0)

        # Simpan waktu update terakhir
        self.last_time = now
        self.last_update_time = now

        return self.x

    # ================================================================
    # [EKF] GETTER — Ambil posisi dan kecepatan yang sudah difilter
    # ================================================================
    def get_position(self):
        """
        Ambil posisi bola yang sudah difilter EKF.

        Returns:
            (px, py): posisi pixel bola yang smooth
        """
        return float(self.x[0]), float(self.x[1])

    def get_velocity(self):
        """
        Ambil estimasi kecepatan bola dari EKF.

        Returns:
            (vx, vy): kecepatan pixel/detik
        """
        return float(self.x[2]), float(self.x[3])

    # ================================================================
    # [EKF] RESET — Reset filter ke kondisi awal
    # Dipanggil saat masuk scan mode atau bola hilang terlalu lama
    # ================================================================
    def reset(self):
        """
        Reset EKF ke kondisi awal (belum terinisialisasi).
        Dipanggil saat bola hilang lebih dari max_predict_time
        atau saat transisi mode scan → track.
        """
        # [EKF] Reset state ke tengah frame
        self.x = np.array([320.0, 240.0, 0.0, 0.0])
        self.P = np.eye(4) * 500.0
        self.initialized = False
        self.last_time = None
        self.last_update_time = None

    # ================================================================
    # [EKF] CHECK — Apakah perlu di-reset karena terlalu lama predict
    # ================================================================
    def should_reset(self):
        """
        Cek apakah filter perlu di-reset karena sudah terlalu lama
        tidak menerima measurement baru (bola tidak terdeteksi).

        Returns:
            True jika perlu reset, False jika masih valid
        """
        if not self.initialized:
            return False
        if self.last_update_time is None:
            return False
        elapsed = time.time() - self.last_update_time
        return elapsed > self.max_predict_time
