# Analisis: Apa yang Sudah Ada di `aroc26` vs Apa yang Kurang

---

## ✅ Yang Sudah Ada di `aroc26`

### 1. `headControl26/vision.py` — Deteksi Bola (YOLO)
Sudah berfungsi penuh. Subscribe ke `/image_raw`, jalankan YOLO OpenVINO, deteksi bola dengan confidence terbaik, lalu **publish koordinat `cx, cy` ke topic `/obj_detect`**.

### 2. `headControl26/mainHeadControl.py` — Head Tracking dengan PID
Subscribe ke `/obj_detect`, lalu gerakkan kepala robot (pan & tilt) agar bola selalu di tengah frame. Menggunakan PID dari `motionPID.py`.

### 3. `headControl26/motionPID.py` — PID Controller
Implementasi PID lengkap, sudah dipakai oleh `mainHeadControl`.

### 4. `headControl26/ballEKF.py` — Extended Kalman Filter untuk bola
Sudah ada filtering posisi bola.

### 5. `buttonHandler26/buttonHandler.py` — Handler Tombol OpenCR
Subscribe tombol fisik robot, bisa toggle mode walking/soccer, init pose, start/stop berjalan.

### 6. Framework ROBOTIS OP3 lengkap
`op3_walking_module`, `op3_action_module`, `op3_online_walking_module`, dst.
Semua package C++ sudah ada, termasuk message definitions (`WalkingParam`, `StartAction`, dll).

---

## ❌ Yang Belum Ada / Kurang

Berikut komponen yang **belum ada** namun dibutuhkan untuk fitur **orbit ke bola + posisi sejajar gawang + tendang**:

### 1. ❌ `motion_module.py` (ROS2 wrapper gerak badan)
Di `StillUseless` ada `motion_module.py` yang jadi **jembatan antara logika program dan robot**. Di `aroc26` belum ada. File ini berisi fungsi-fungsi seperti:
- `WalkingCommand(start/stop)`
- `Motion_WalkingParams(x, y, angle, period)` — **ini kunci utama untuk orbit**
- `MotionActionNum(page)` — untuk tendangan
- `LED_RGB`, `Buzzer` — feedback status

Tanpa ini, tidak ada cara mudah untuk memerintah robot berjalan dengan parameter tertentu dari Python.

---

### 2. ❌ `data_flow_handler.py` (Shared Data / ROS Handler)
Di `StillUseless` ini adalah **pusat data global** yang menyimpan semua state robot (posisi bola dari kamera, data IMU, state kepala, dll) yang bisa diakses oleh semua modul. Di `aroc26` belum ada, sehingga antar-node tidak bisa berbagi data dengan mudah.

---

### 3. ❌ `task_control.py` / State Machine Utama
Ini adalah **otak utama robot** — state machine yang mengatur alur:
```
IDLE → STANDBY → SEARCH_BALL → APPROACH_BALL → ORBIT → ALIGN_TO_GOAL → KICK → kembali ke SEARCH
```
Di `aroc26` sama sekali belum ada. `KickNRush.py` yang ada masih kosong.

---

### 4. ❌ Logic Orbit ke Bola
Fitur inti yang dibutuhkan. Belum ada sama sekali. Konsepnya:
- Robot berjalan **melingkar di sekitar bola** menggunakan kombinasi `x_move_amplitude` (maju) + `y_move_amplitude` (geser samping) + `angle_move_amplitude` (rotasi)
- Sambil orbit, terus cek sudut antara **arah badan robot → bola → gawang**
- Berhenti orbit saat sudut tersebut sudah dalam rentang yang pas untuk menendang

---

### 5. ❌ Logic Alignment ke Gawang (Goal Alignment)
Untuk mengetahui arah gawang, dibutuhkan salah satu dari:
- **IMU yaw** (gyro terintegrasi) sebagai referensi arah — data sudah tersedia di topic `/robotis/open_cr/imu` tapi belum diproses
- Atau **deteksi visual gawang** (gawang kuning) menggunakan YOLO/color masking

Belum ada subscriber IMU, belum ada estimasi posisi gawang.

---

### 6. ❌ Logic Approach (Mendekati Bola)
Sebelum orbit, robot harus mendekati bola dulu. Perlu logika:
- Estimasi jarak bola dari ukuran bounding box (ukuran `cx, cy` + lebar box)
- Atur kecepatan maju berdasarkan jarak

Belum ada di `aroc26`.

---

### 7. ❌ Fall Detection + Get Up
Di `StillUseless` ada `isFalling()` yang baca IMU dan trigger action page 122/123 (GetUpFront/Back). Di `aroc26` belum ada, padahal ini **sangat penting** agar robot tidak stuck saat jatuh.

---

### 8. ❌ Node Handler / Entry Point Utama
Di `StillUseless` ada `main_node_handler.py` yang jadi entry point. Di `aroc26`, `KickNRush.py` kosong dan belum ada node utama yang mengorkestrasi semua node.

---

## 📊 Ringkasan Tabel

| Komponen | StillUseless | aroc26 | Keterangan |
|---|---|---|---|
| Deteksi bola (YOLO) | ✅ | ✅ | Sudah ada, bahkan lebih canggih di aroc26 |
| Head tracking PID | ✅ | ✅ | Sudah ada |
| PID Controller | ✅ | ✅ | Sudah ada |
| Motion wrapper (walking/action) | ✅ | ❌ | **Perlu dibuat** |
| Shared data handler | ✅ | ❌ | **Perlu dibuat** |
| State machine utama | ✅ | ❌ | **Perlu dibuat** |
| Logic orbit ke bola | ✅ | ❌ | **Perlu dibuat** |
| Alignment ke gawang | ❌ | ❌ | **Perlu dibuat dari nol** |
| Approach / estimasi jarak | parsial | ❌ | **Perlu dibuat** |
| Fall detection | ✅ | ❌ | **Perlu dibuat** |
| Node handler / entry point | ✅ | ❌ | **Perlu dibuat** |
| Button handler | ✅ | ✅ | Sudah ada |
| Game controller (UDP) | ✅ | ❌ | Opsional untuk kompetisi |

---

## 🗂️ Rencana Struktur yang Perlu Dibuat di `aroc26`

```
aroc26/src/KickNRush/KickNRush/
├── __init__.py
├── motion_module.py        ← wrapper ROS2 untuk walking & action
├── data_handler.py         ← shared data global (bola, IMU, state)
├── approach_control.py     ← logika mendekati bola
├── orbit_control.py        ← logika orbit mengelilingi bola ★
├── goal_alignment.py       ← estimasi arah gawang via IMU/vision ★
├── fall_detection.py       ← deteksi jatuh & bangkit
├── task_control.py         ← state machine utama (otak robot) ★
└── main.py                 ← entry point / node handler
```
