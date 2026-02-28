# 🪐 Tutorial Motion Orbit dari Nol — AROC26
> Berdasarkan kode nyata di `aroc26/src/KickNRush/` dan `aroc26/src/headControl26/`

---

## 🗺️ PETA BESAR — Apa yang Akan Kamu Buat

```
📁 aroc26/src/KickNRush/KickNRush/
├── motion_orbit.py   ← 🪐 KELAS ORBIT (hitung parameter gerak)
├── task_control.py   ← 🧠 STATE MACHINE (kapan orbit dijalankan)
├── motion_crab.py    ← 🦀 geser murni (referensi)
└── main.py           ← 🚀 entry point node

📁 aroc26/src/headControl26/headControl26/
└── mainHeadControl.py ← 👁️ kepala otomatis track bola (SUDAH JALAN SENDIRI)
```

> ✅ File yang **perlu kamu buat/edit**: hanya `motion_orbit.py` dan `task_control.py`
> ✅ File lain (`vision.py`, `mainHeadControl.py`, `ballEKF.py`) **sudah jalan sendiri**

---

## 🍼 Konsep Orbit dalam 1 Paragraf Bayi

Robot berdiri menghadap bola. Supaya robot bisa **jalan melingkar** sambil **terus nghadap bola**:
- `y_move_amplitude` → robot **geser ke samping** (kayak kepiting 🦀)
- `angle_move_amplitude` → robot **putar badannya** ke arah bola
- Kepala tracking otomatis dari `headControl26` (tidak perlu kamu urus)

Hasilnya: robot jalan seperti **planet ngorbit matahari** ☀️ — bola selalu ada di depan robot!

---

## 🏗️ ARSITEKTUR SISTEM AROC26

### Cara kerja antar-node

```
[vision.py]          → publish /obj_detect         → [mainHeadControl.py]
                                                           ↓ kepala gerak otomatis
[vision.py]          → publish /obj_detect          → [task_control.py]
[vision.py]          → publish /obj_detect_ball_bbox → [task_control.py]
[OpenCR button]      → publish /robotis/open_cr/button → [task_control.py]
[IMU]                → publish /robotis/open_cr/imu    → [task_control.py]

[task_control.py]    → publish /robotis/walking/set_params → [op3_walking_module]
[task_control.py]    → publish /robotis/walking/command    → [op3_walking_module]
[task_control.py]    → publish /robotis/enable_ctrl_module → [robotis_controller]
[task_control.py]    → publish /robotis/action/page_num   → [op3_action_module]
```

### Topics penting (dari `robot_soccer_topics_and_kicking_guide.txt`)

| Topic | Tipe | Fungsi |
|---|---|---|
| `/robotis/walking/set_params` | `WalkingParam` | Set kecepatan & arah jalan |
| `/robotis/walking/command` | `String` | `"start"` / `"stop"` |
| `/robotis/enable_ctrl_module` | `String` | Ganti module aktif |
| `/robotis/action/page_num` | `Int32` | Jalankan gerakan (tendang, bangun) |
| `/obj_detect` | `String` | Posisi bola `"cx,cy"` dari kamera |
| `/obj_detect_ball_bbox` | `String` | Bbox bola `"cx,cy,w,h"` |
| `/robotis/open_cr/button` | `String` | Tombol OpenCR (`"start"`, `"mode"`) |
| `/robotis/open_cr/imu` | `Imu` | Data IMU untuk fall detection |

---

## 📐 Parameter Walking yang Kamu Kendalikan

```
WalkingParam
├── x_move_amplitude     → maju/mundur (meter)   | + = maju, - = mundur
├── y_move_amplitude     → geser kiri/kanan (m)  | + = kiri, - = kanan
├── angle_move_amplitude → putar badan (radian)  | + = kiri, - = kanan
└── period_time          → periode 1 langkah (s) | 0.50–0.65 biasanya
```

### Rumus orbit:

```
Orbit KANAN:
  y     = -0.030  (geser kanan)
  angle = +0.122  (putar kiri, supaya tetap nghadap bola) ← 7° dalam radian

Orbit KIRI:
  y     = +0.030  (geser kiri)
  angle = -0.122  (putar kanan, supaya tetap nghadap bola)
```

> **Kenapa y dan angle berlawanan tanda?**
> Bayangkan kamu jalan ke kanan sambil terus menghadap tembok — kamu harus putar badan ke kiri!

---

## 📋 State Machine yang Sudah Ada

```
IDLE → SEARCH → APPROACH → ORBIT → ADJUST → KICK
                               ↑
                          KO ini yang kamu fokuskan!

Kapan masuk ORBIT:
  State APPROACH → ball_width >= BALL_WIDTH_CLOSE (bola cukup dekat)

Kapan keluar ORBIT:
  → SEARCH  : bola hilang > 1.5 detik
  → ADJUST  : robot sudah sejajar gawang (dari GoalAlignment)
```

---

## 🚀 LANGKAH DEMI LANGKAH

---

### LANGKAH 1 — Pahami file yang sudah ada

Buka dan baca dulu file ini sebelum mulai coding:

```bash
# File kelas orbit yang sudah ada (referensi utama kamu):
aroc26/src/KickNRush/KickNRush/motion_orbit.py

# State machine yang sudah jalan:
aroc26/src/KickNRush/KickNRush/task_control.py

# Referensi kelas crab (mirip orbit tapi tanpa rotasi):
aroc26/src/KickNRush/KickNRush/motion_crab.py
```

**Poin penting dari `motion_orbit.py` yang sudah ada:**

```python
ORBIT_Y_SPEED     = 0.030   # m — kecepatan geser samping
ORBIT_ANGLE_SPEED = 7.0     # deg — rotasi per langkah (dalam DERAJAT!)
ORBIT_X_FORWARD   = 0.010   # m — maju kecil agar radius tidak membesar
ORBIT_PERIOD      = 0.55    # s — period satu langkah
```

> ⚠️ **PENTING**: `motion_orbit.py` return angle dalam **DERAJAT**.
> Konversi ke radian dilakukan di `task_control.py`:
> ```python
> param.angle_move_amplitude = angle_deg * 3.14159 / 180.0
> ```

---

### LANGKAH 2 — Buat file `motion_orbit.py` dari nol

Buat file baru di `aroc26/src/KickNRush/KickNRush/motion_orbit.py`:

```python
# motion_orbit.py
# Kelas untuk menghitung parameter walking saat orbit mengelilingi bola
# ORBIT = geser samping (y) + putar badan (angle) secara bersamaan
# Publisher WalkingParam ada di task_control.py, BUKAN di sini

from rclpy.node import Node

# ── Konstanta tuning ──────────────────────────────────────────
ORBIT_Y_SPEED     = 0.030  # m   — kecepatan geser samping
ORBIT_ANGLE_SPEED = 7.0    # deg — rotasi badan per langkah (DERAJAT!)
ORBIT_X_FORWARD   = 0.010  # m   — maju sedikit agar tidak menjauh
ORBIT_PERIOD      = 0.55   # s   — periode satu langkah


class MotionOrbit:
    """
    Hitung parameter walking untuk orbit mengelilingi bola.

    Prinsip:
      Orbit KANAN → y negatif + angle positif
      Orbit KIRI  → y positif + angle negatif

    PENTING: angle dikembalikan dalam DERAJAT.
    Konversi ke radian dilakukan di task_control.py sebelum publish.
    """

    def __init__(self, node: Node):
        self.node       = node
        self._direction = "right"         # arah orbit default
        self._x         = ORBIT_X_FORWARD
        self._y         = -ORBIT_Y_SPEED  # default kanan (y negatif)
        self._angle     = ORBIT_ANGLE_SPEED  # default kanan (angle positif)
        self._period    = ORBIT_PERIOD
        self.node.get_logger().info("[MotionOrbit] Init — default: right")

    def set_direction(self, direction: str):
        """
        Set arah orbit: 'left' atau 'right'

        Orbit kanan: y negatif (geser kanan) + angle positif (putar kiri)
        Orbit kiri : y positif (geser kiri)  + angle negatif (putar kanan)
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
                f"[MotionOrbit] Arah tidak dikenal: '{direction}'. Gunakan 'left' atau 'right'.")
            return
        self.node.get_logger().info(
            f"[MotionOrbit] Arah: {direction} | y={self._y:.3f}m | angle={self._angle:.1f}deg")

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
            f"[MotionOrbit] Custom → x={x} y={y} angle={angle}deg period={period}s")

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
```

---

### LANGKAH 3 — Pahami bagian ORBIT di `task_control.py`

Ini adalah bagian state ORBIT yang sudah ada di `task_control.py`. **Baca dan pahami dulu:**

```python
def _state_orbit(self, ball_lost: bool):
    """Orbit mengelilingi bola. Arah dari GoalAlignment (IMU+YOLO)."""

    # 1. Kalau bola hilang → balik cari bola
    if ball_lost:
        self._stop_walking()
        self._set_state(State.SEARCH)
        return

    # 2. Kalau sudah sejajar gawang → masuk ADJUST
    if self.alignment.is_aligned():
        self._stop_walking()
        self._set_state(State.ADJUST)
        return

    # 3. Antisipasi head_control_module override walking_module
    self._enable_walking_module_soft()

    # 4. Tanya GoalAlignment: orbit ke kiri atau kanan?
    direction = self.alignment.get_orbit_direction()  # → "left" atau "right"
    self.orbit.set_direction(direction)
    x, y, angle_deg, period = self.orbit.get_params()

    # 5. Konversi angle dari derajat ke radian lalu publish
    param = WalkingParam()
    param.x_move_amplitude     = x
    param.y_move_amplitude     = y
    param.angle_move_amplitude = angle_deg * 3.14159 / 180.0  # ← konversi di sini!
    param.period_time          = period
    self._pub_walking_param.publish(param)
    self._walking_start()
```

---

### LANGKAH 4 — Buat versi sederhana `task_control.py` dari nol

Jika kamu ingin buat `task_control.py` dari nol (tanpa `GoalAlignment`), ini templatenya:

```python
# task_control.py — VERSI SEDERHANA (tanpa GoalAlignment)
# Orbit tanpa cek arah gawang — robot orbit ke satu arah saja

import time
import math
from enum import Enum, auto

from std_msgs.msg import String, Int32
from sensor_msgs.msg import Imu
from op3_walking_module_msgs.msg import WalkingParam

from .motion_orbit import MotionOrbit

# ── Konstanta ─────────────────────────────────────────────────
FRAME_W              = 640
FRAME_H              = 480
BALL_WIDTH_CLOSE     = 100   # px — bola dekat → mulai orbit
BALL_LOST_TIMEOUT    = 1.5   # s  — timeout bola hilang
FALL_THRESHOLD       = 7.0   # m/s²

ACTION_KICK_RIGHT    = 83
ACTION_KICK_LEFT     = 84
ACTION_GETUP_FRONT   = 122
ACTION_GETUP_BACK    = 123


class State(Enum):
    IDLE     = auto()
    SEARCH   = auto()
    APPROACH = auto()
    ORBIT    = auto()
    GET_UP   = auto()


class TaskControl:
    def __init__(self, node):
        self.node        = node
        self.state       = State.IDLE
        self._prev_state = State.IDLE

        # Inisialisasi modul orbit
        self.orbit = MotionOrbit(node)

        # Publisher
        self._pub_walking_cmd   = node.create_publisher(String, '/robotis/walking/command', 10)
        self._pub_walking_param = node.create_publisher(WalkingParam, '/robotis/walking/set_params', 10)
        self._pub_action        = node.create_publisher(Int32, '/robotis/action/page_num', 10)
        self._pub_module        = node.create_publisher(String, '/robotis/enable_ctrl_module', 10)

        # Subscriber
        node.create_subscription(String, '/obj_detect_ball_bbox', self._ball_bbox_callback, 10)
        node.create_subscription(String, '/robotis/open_cr/button', self._button_callback, 10)
        node.create_subscription(Imu,    '/robotis/open_cr/imu',    self._imu_callback, 10)

        # Data internal
        self._ball_cx        = None
        self._ball_cy        = None
        self._ball_width     = None
        self._ball_detected  = False
        self._last_ball_time = 0.0
        self._imu_accel_x    = 0.0
        self._task_timer     = time.time()
        self._last_module_pub = 0.0

        node.get_logger().info("[TaskControl] Siap — state: IDLE")

    # ── CALLBACKS ─────────────────────────────────────────────

    def _ball_bbox_callback(self, msg: String):
        """Terima bbox bola: format 'cx,cy,w,h'"""
        try:
            parts = msg.data.strip().split(',')
            if len(parts) >= 3:
                self._ball_cx       = float(parts[0])
                self._ball_cy       = float(parts[1])
                self._ball_width    = float(parts[2])
                self._ball_detected = True
                self._last_ball_time = time.time()
        except ValueError:
            pass

    def _button_callback(self, msg: String):
        """Tombol OpenCR: 'start' mulai | 'mode' stop"""
        btn = msg.data.strip()
        if btn == "start" and self.state == State.IDLE:
            self._enable_walking_module()
            self._set_state(State.SEARCH)
        elif btn == "mode":
            self._stop_walking()
            self._set_state(State.IDLE)

    def _imu_callback(self, msg: Imu):
        self._imu_accel_x = msg.linear_acceleration.x

    # ── MAIN LOOP ─────────────────────────────────────────────

    def update(self):
        """Dipanggil 20 Hz dari main.py"""
        # Fall detection — cek di semua state kecuali IDLE dan GET_UP
        if self.state not in (State.IDLE, State.GET_UP):
            if abs(self._imu_accel_x) > FALL_THRESHOLD:
                self._prev_state = self.state
                self._set_state(State.GET_UP)
                return

        ball_lost = (time.time() - self._last_ball_time) > BALL_LOST_TIMEOUT

        if   self.state == State.IDLE:     pass
        elif self.state == State.SEARCH:   self._state_search(ball_lost)
        elif self.state == State.APPROACH: self._state_approach(ball_lost)
        elif self.state == State.ORBIT:    self._state_orbit(ball_lost)
        elif self.state == State.GET_UP:   self._state_get_up()

    # ── STATE HANDLERS ────────────────────────────────────────

    def _state_search(self, ball_lost: bool):
        """Putar badan cari bola."""
        if self._ball_detected and not ball_lost:
            self._set_state(State.APPROACH)
            return
        # Putar badan ke kiri 10 derajat
        param = WalkingParam()
        param.x_move_amplitude     = 0.0
        param.y_move_amplitude     = 0.0
        param.angle_move_amplitude = 10.0 * math.pi / 180.0
        param.period_time          = 0.55
        self._pub_walking_param.publish(param)
        self._walking_start()

    def _state_approach(self, ball_lost: bool):
        """Maju ke bola sampai cukup dekat."""
        if ball_lost:
            self._stop_walking()
            self._set_state(State.SEARCH)
            return

        dist = self._ball_width if self._ball_width is not None else 0

        if dist >= BALL_WIDTH_CLOSE:
            # Bola sudah dekat → mulai orbit
            self._stop_walking()
            self.orbit.set_direction("right")  # orbit default ke kanan
            self._set_state(State.ORBIT)
            return

        # Maju ke bola
        param = WalkingParam()
        param.x_move_amplitude     = 0.030
        param.y_move_amplitude     = 0.0
        param.angle_move_amplitude = 0.0
        param.period_time          = 0.55
        self._pub_walking_param.publish(param)
        self._walking_start()

    def _state_orbit(self, ball_lost: bool):
        """Orbit mengelilingi bola."""
        if ball_lost:
            self._stop_walking()
            self._set_state(State.SEARCH)
            return

        # Antisipasi module override
        self._enable_walking_module_soft()

        # Ambil parameter dari MotionOrbit
        x, y, angle_deg, period = self.orbit.get_params()

        param = WalkingParam()
        param.x_move_amplitude     = x
        param.y_move_amplitude     = y
        param.angle_move_amplitude = angle_deg * math.pi / 180.0  # deg → rad
        param.period_time          = period
        self._pub_walking_param.publish(param)
        self._walking_start()

    def _state_get_up(self):
        """Bangun setelah jatuh."""
        self._stop_walking()
        self._enable_action_module()
        time.sleep(0.3)
        if self._imu_accel_x > FALL_THRESHOLD:
            self._do_action(ACTION_GETUP_BACK)
        else:
            self._do_action(ACTION_GETUP_FRONT)
        time.sleep(4.0)
        self._enable_walking_module()
        self._set_state(self._prev_state)

    # ── UTILITY ───────────────────────────────────────────────

    def _set_state(self, new_state: State):
        self.node.get_logger().info(
            f"[TaskControl] {self.state.name} → {new_state.name}")
        self.state       = new_state
        self._task_timer = time.time()

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
        """Publish walking_module tanpa sleep — aman dipanggil tiap loop 20Hz."""
        now = time.time()
        if (now - self._last_module_pub) > 1.0:
            msg = String(); msg.data = 'walking_module'
            self._pub_module.publish(msg)
            self._last_module_pub = now

    def _enable_action_module(self):
        msg = String(); msg.data = 'action_module'
        self._pub_module.publish(msg)
        time.sleep(0.3)
```

---

### LANGKAH 5 — Buat `main.py` dari nol

```python
# main.py — Entry point node KickNRush / orbit demo
import rclpy
from rclpy.node import Node
from .task_control import TaskControl

LOOP_HZ = 20  # 20 Hz = update tiap 50ms


class OrbitDemoNode(Node):
    def __init__(self):
        super().__init__('orbit_demo_node')
        self.get_logger().info("=== Orbit Demo Node ===")
        self.task  = TaskControl(self)
        self.timer = self.create_timer(1.0 / LOOP_HZ, self._loop)
        self.get_logger().info(f"Loop @ {LOOP_HZ} Hz — Tekan START di OpenCR!")

    def _loop(self):
        try:
            self.task.update()
        except Exception as e:
            self.get_logger().error(f"[OrbitDemo] Error: {e}")


def main():
    rclpy.init()
    node = OrbitDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Dihentikan.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

### LANGKAH 6 — Pastikan `setup.py` benar

Cek file `aroc26/src/KickNRush/setup.py`:

```python
from setuptools import setup

package_name = 'KickNRush'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'KickNRush = KickNRush.main:main',
        ],
    },
)
```

---

### LANGKAH 7 — Build dan jalankan

```bash
# Di root workspace aroc26
cd ~/aroc26

# Build hanya package yang dibutuhkan
colcon build --packages-select KickNRush

# Source workspace
source install/setup.bash

# Jalankan (pastikan headControl26 sudah jalan duluan!)
ros2 run KickNRush KickNRush
```

Atau pakai launch file:
```bash
ros2 launch KickNRush KickNRush.launch.py
```

---

### LANGKAH 8 — Test tanpa robot (debug dari terminal)

```bash
# Terminal 1 — Simulasi bola ketemu
ros2 topic pub /obj_detect_ball_bbox std_msgs/String \
  "{data: '320,400,120,100'}" --rate 10
# format: cx, cy, width, height (px)

# Terminal 2 — Tekan tombol start (simulasi)
ros2 topic pub /robotis/open_cr/button std_msgs/String \
  "{data: 'start'}" --once

# Terminal 3 — Monitor walking params yang dikirim
ros2 topic echo /robotis/walking/set_params

# Terminal 4 — Monitor state machine via log
ros2 topic echo /rosout
```

---

## 🎛️ TUNING PARAMETER ORBIT

Semua konstanta ada di `motion_orbit.py`. Ubah satu per satu:

| Parameter | Default | Efek naik | Efek turun | Rekomendasi awal |
|---|---|---|---|---|
| `ORBIT_Y_SPEED` | `0.030` | Orbit lebih cepat | Orbit lebih pelan | `0.025–0.035` |
| `ORBIT_ANGLE_SPEED` | `7.0` deg | Putar lebih cepat | Putar lebih pelan | `5.0–10.0` |
| `ORBIT_X_FORWARD` | `0.010` | Makin mendekat | Diam di tempat | `0.0–0.015` |
| `ORBIT_PERIOD` | `0.55` | Langkah lebih lambat | Langkah lebih cepat | `0.50–0.65` |

> **Cara tuning yang benar:**
> 1. Mulai semua dari nilai default
> 2. Ubah **hanya satu parameter** per percobaan
> 3. Amati efeknya di robot
> 4. Catat nilai yang bagus

---

## 🐛 TROUBLESHOOTING

| Gejala | Penyebab | Solusi |
|---|---|---|
| Robot tidak geser | `ORBIT_Y_SPEED` terlalu kecil | Naikan ke `0.040` |
| Robot tidak putar badan | `ORBIT_ANGLE_SPEED` = 0 | Set ke `7.0` |
| Robot menjauh dari bola | `ORBIT_X_FORWARD` negatif | Set ke `+0.010` |
| Robot jatuh saat orbit | `ORBIT_PERIOD` terlalu kecil | Naikan ke `0.65` |
| Kepala tidak tracking | `headControl26` belum jalan | Jalankan headControl26 dulu |
| State tidak berubah | `ball_width` tidak ter-update | Cek topic `/obj_detect_ball_bbox` |
| `walking_module` di-override | `head_control_module` rebutan | Gunakan `_enable_walking_module_soft()` |

---

## 📊 DIAGRAM ALUR LENGKAP

```
[Tekan START]
      ↓
   SEARCH ──────── bola ketemu ──────→ APPROACH
      ↑                                    │
      │                              bola dekat
      │                             (width≥100px)
      │                                    ↓
      └───── bola hilang ─────────── ORBIT 🪐
                                          │
                              sejajar gawang → ADJUST → KICK
                                          │
                              jatuh → GET_UP → kembali

Di state ORBIT setiap 50ms (20Hz):
┌─────────────────────────────────────────────┐
│ 1. Cek bola masih ada?                      │
│ 2. Aktifkan walking_module (soft publish)   │
│ 3. orbit.get_params() → x, y, angle, period│
│ 4. Konversi angle: deg × π/180 → rad       │
│ 5. Publish WalkingParam                     │
│ 6. Publish walking/command "start"          │
└─────────────────────────────────────────────┘
```

---

## ✅ CHECKLIST IMPLEMENTASI

- [ ] Buat `motion_orbit.py` dengan class `MotionOrbit`
- [ ] Definisikan konstanta: `ORBIT_Y_SPEED`, `ORBIT_ANGLE_SPEED`, `ORBIT_X_FORWARD`, `ORBIT_PERIOD`
- [ ] Implementasi `set_direction("left"/"right")`
- [ ] Implementasi `get_params()` → return `(x, y, angle_deg, period)`
- [ ] Di `task_control.py`: buat state `ORBIT` di Enum
- [ ] Di `task_control.py`: subscribe `/obj_detect_ball_bbox`
- [ ] Di `task_control.py`: trigger ORBIT dari state APPROACH saat bola dekat
- [ ] Di `_state_orbit()`: konversi angle deg → rad sebelum publish
- [ ] Di `_state_orbit()`: handle ball_lost → kembali ke SEARCH
- [ ] Build dengan `colcon build`
- [ ] Test dengan simulasi topic dari terminal
- [ ] Tuning parameter di robot nyata

---

*Generated: 2026-02-28 | Berdasarkan kode nyata aroc26/src/KickNRush & headControl26*
