# Logika Orbit ke Bola — Penjelasan Bahasa Bayi 👶

---

## Analogi Sederhana

Bayangin kamu berdiri di lapangan, ada **bola di depan kamu**, dan ada **gawang di kejauhan**. Kamu mau nendang bola masuk gawang.

**Masalahnya:** posisi kamu sekarang **tidak lurus dengan gawang**. Bola ada di depan, tapi gawang ada di pojok kanan.

```
GAWANG
  │
  │          ← gawang ada di sini
  │
  
  ⚽  ← bola

      🤖  ← kamu (robot) ada di sini, tidak lurus ke gawang
```

Solusinya? Kamu **jalan melingkar mengelilingi bola** pelan-pelan, sampai posisi kamu **lurus ke gawang melewati bola**.

```
GAWANG
  │
  │
  │
  ⚽  ← bola
  │
  🤖  ← sekarang posisi kamu sudah lurus! siap tendang!
```

Itu namanya **ORBIT** 🌍

---

## Robot Jalan Itu Punya 3 "Tombol"

Robot OP3 bisa dikasih perintah jalan dengan 3 parameter:

```
x_move  → maju atau mundur
           positif = maju, negatif = mundur

y_move  → geser kiri atau kanan (kayak crab walk)
           positif = geser kiri, negatif = geser kanan

angle   → mutar badannya
           positif = mutar kiri, negatif = mutar kanan
```

Kalau biasanya jalan biasa itu cuma pakai `x_move`, orbit itu pakai **kombinasi `y_move` + `angle`** secara bersamaan.

---

## Contoh Konkret: Orbit ke Kanan

Misalnya gawang ada di kanan bola, dan robot perlu orbit ke kanan mengelilingi bola:

```python
# Orbit ke KANAN mengelilingi bola
x_move  =  0.01   # maju sedikit biar tidak mundur
y_move  = -0.03   # geser ke kanan
angle   = -5.0    # sambil mutar badan ke kiri (menghadap bola terus)
```

Bayanginnya gini:

```
     ⚽ bola

🤖 ──────────► (geser kanan)
     ↺         (mutar badan kiri biar tetap lihat bola)
```

Hasilnya robot bergerak **melingkar ke kanan** sambil **terus menghadap bola**.

---

## Gimana Tau Sudah Lurus ke Gawang?

Ada 2 cara:

### Cara 1: Pakai IMU (Gyroscope/Compass)
Robot punya sensor IMU yang bisa kasih tau **robot menghadap ke arah mana** (dalam derajat, namanya **yaw**).

```
- Waktu init/berdiri awal, kita anggap arah itu = 0 derajat
- Kalau gawang ada di depan lurus = target yaw = 0 derajat
- Kalau robot sekarang yaw = 30 derajat → berarti masih perlu orbit
- Orbit terus sampai yaw mendekati 0 derajat
```

```python
yaw_sekarang = imu_data.yaw     # baca dari sensor
yaw_target   = 0.0              # arah gawang

error = yaw_target - yaw_sekarang

if abs(error) < 5.0:            # toleransi 5 derajat
    print("SUDAH LURUS! SIAP TENDANG!")
else:
    # masih perlu orbit
    if error > 0:
        orbit_ke_kiri()
    else:
        orbit_ke_kanan()
```

### Cara 2: Pakai Visual (Deteksi Gawang)
Kalau bisa deteksi gawang pakai kamera, cukup cek apakah **gawang ada di tengah-tengah frame** saat robot lihat ke depan.

```python
gawang_cx = 320   # gawang terdeteksi di pixel x = 320
frame_tengah = 320  # tengah frame (640px lebar)

error = frame_tengah - gawang_cx

if abs(error) < 30:   # toleransi 30 pixel
    print("GAWANG SUDAH DI TENGAH! SIAP TENDANG!")
```

---

## Gimana Tau Bola Sudah Dekat Untuk Ditendang?

Dari deteksi YOLO, kita dapat **bounding box** bola: koordinat + lebar + tinggi kotak.

```
Bola jauh  → kotak kecil  (lebar = 20px)
Bola dekat → kotak besar  (lebar = 80px)
```

```python
ball_width = bbox[2]   # lebar bounding box bola dalam pixel

if ball_width > 70:
    print("BOLA SUDAH DEKAT! Mulai orbit")
elif ball_width > 100:
    print("BOLA SANGAT DEKAT! Siap tendang")
else:
    print("BOLA MASIH JAUH, maju dulu")
```

---

## Kode Lengkap: `orbit_control.py`

```python
import math

class OrbitControl:
    """
    Kelas untuk mengatur orbit robot mengelilingi bola.
    
    Cara pakai:
        orbit = OrbitControl()
        x, y, angle = orbit.calculate(yaw_sekarang, yaw_target)
        motion.set_walking_params(x=x, y=y, angle=angle)
    """

    def __init__(self):
        self.orbit_speed = 0.03   # kecepatan geser (y_move) saat orbit
        self.orbit_angle = 5.0    # derajat rotasi per langkah
        self.x_forward  = 0.01   # sedikit maju biar tidak mundur
        self.tolerance  = 5.0    # toleransi sudut dalam derajat

    def is_aligned(self, yaw_sekarang, yaw_target):
        """Cek apakah robot sudah lurus ke gawang"""
        error = yaw_target - yaw_sekarang
        return abs(error) < self.tolerance

    def calculate(self, yaw_sekarang, yaw_target):
        """
        Hitung parameter walking untuk orbit.
        Return: (x_move, y_move, angle_move)
        """
        error = yaw_target - yaw_sekarang

        if abs(error) < self.tolerance:
            # Sudah lurus, berhenti orbit
            return 0.0, 0.0, 0.0

        if error > 0:
            # Perlu orbit ke KIRI
            # → geser kiri + mutar kanan (biar tetap hadap bola)
            y_move = +self.orbit_speed
            angle  = -self.orbit_angle
        else:
            # Perlu orbit ke KANAN
            # → geser kanan + mutar kiri (biar tetap hadap bola)
            y_move = -self.orbit_speed
            angle  = +self.orbit_angle

        return self.x_forward, y_move, angle
```

---

## Kode Lengkap: `motion_module.py`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from robotis_controller_msgs.msg import SyncWriteItem

class MotionModule(Node):
    """
    Wrapper untuk semua perintah gerak robot OP3.
    Satu tempat untuk kontrol jalan, action, LED, buzzer.
    """

    def __init__(self):
        super().__init__('motion_module')

        # Publisher untuk perintah walking
        self.pub_walking_cmd = self.create_publisher(
            String, '/robotis/walking/command', 10)

        # Publisher untuk set parameter walking
        self.pub_walking_param = self.create_publisher(
            String, '/robotis/walking/set_params', 10)

        # Publisher untuk action (tendangan, berdiri, dll)
        self.pub_action = self.create_publisher(
            Int32, '/robotis/action/page_num', 10)

    def start_walking(self):
        """Mulai jalan"""
        msg = String()
        msg.data = 'start'
        self.pub_walking_cmd.publish(msg)

    def stop_walking(self):
        """Berhenti jalan"""
        msg = String()
        msg.data = 'stop'
        self.pub_walking_cmd.publish(msg)

    def set_walking_params(self, x=0.0, y=0.0, angle=0.0):
        """
        Set parameter jalan.
        x     : maju/mundur (meter)
        y     : geser kiri/kanan (meter)
        angle : rotasi (derajat)
        """
        # Format sesuai ROBOTIS OP3 walking module
        param_str = f'x_move_amplitude: {x}, y_move_amplitude: {y}, angle_move_amplitude: {angle}'
        msg = String()
        msg.data = param_str
        self.pub_walking_param.publish(msg)

    def do_action(self, page_num):
        """
        Jalankan action berdasarkan nomor halaman.
        Contoh page:
          12 = tendang kanan
          13 = tendang kiri
          82 = berdiri dari jatuh depan
          83 = berdiri dari jatuh belakang
        """
        self.stop_walking()  # berhenti jalan dulu sebelum action
        msg = Int32()
        msg.data = page_num
        self.pub_action.publish(msg)
```

---

## Kode Lengkap: `task_control.py` (State Machine Utama)

```python
from enum import Enum

class State(Enum):
    SEARCH   = 0   # cari bola
    APPROACH = 1   # dekati bola
    ORBIT    = 2   # orbit mengelilingi bola
    KICK     = 3   # tendang

class TaskControl:
    """
    Otak utama robot. Mengatur kapan robot harus:
    cari bola → dekati → orbit → tendang → ulang
    """

    def __init__(self, motion, orbit_control, data_handler):
        self.motion        = motion
        self.orbit         = orbit_control
        self.data          = data_handler
        self.state         = State.SEARCH
        self.yaw_target    = 0.0   # arah gawang dalam derajat

    def update(self):
        """Dipanggil terus-menerus (loop utama)"""

        # Ambil data terbaru
        ball_detected = self.data.ball_detected
        ball_cx       = self.data.ball_cx       # posisi x bola di frame
        ball_width    = self.data.ball_width     # lebar bounding box bola
        yaw_current   = self.data.imu_yaw        # arah robot sekarang

        # ── STATE: CARI BOLA ──────────────────────────────
        if self.state == State.SEARCH:
            if ball_detected:
                print("Bola ketemu! Mulai dekati...")
                self.motion.start_walking()
                self.state = State.APPROACH
            else:
                # putar di tempat untuk cari bola
                self.motion.set_walking_params(x=0.0, y=0.0, angle=10.0)
                self.motion.start_walking()

        # ── STATE: DEKATI BOLA ────────────────────────────
        elif self.state == State.APPROACH:
            if not ball_detected:
                self.state = State.SEARCH
                return

            if ball_width > 70:
                # sudah cukup dekat, mulai orbit
                print("Sudah dekat! Mulai orbit...")
                self.state = State.ORBIT
            else:
                # masih jauh, maju terus
                self.motion.set_walking_params(x=0.03, y=0.0, angle=0.0)

        # ── STATE: ORBIT ──────────────────────────────────
        elif self.state == State.ORBIT:
            if not ball_detected:
                self.state = State.SEARCH
                return

            if self.orbit.is_aligned(yaw_current, self.yaw_target):
                # sudah lurus ke gawang!
                print("Sudah lurus ke gawang! Siap tendang!")
                self.motion.stop_walking()
                self.state = State.KICK
            else:
                # orbit terus
                x, y, angle = self.orbit.calculate(yaw_current, self.yaw_target)
                self.motion.set_walking_params(x=x, y=y, angle=angle)

        # ── STATE: TENDANG ────────────────────────────────
        elif self.state == State.KICK:
            print("TENDANG!")
            self.motion.do_action(12)   # page 12 = tendang kanan
            self.state = State.SEARCH   # setelah tendang, cari bola lagi
```

---

## Alur Lengkap (Rangkuman Visual)

```
START
  │
  ▼
[SEARCH] ──── bola ketemu? ──NO──► putar cari bola
  │ YES
  ▼
[APPROACH] ── sudah dekat? ──NO──► maju ke bola
  │ YES
  ▼
[ORBIT] ───── sudah lurus? ──NO──► set y_move + angle (orbit)
  │ YES
  ▼
[KICK] ──────────────────────────► tendang! → kembali ke SEARCH
```
