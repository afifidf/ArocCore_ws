# KickNRush — Dokumentasi Sistem AROC26

Dokumentasi lengkap arsitektur, alur kerja, dan cara penggunaan package `KickNRush` untuk robot humanoid ROBOTIS OP3.

---

## Daftar Isi

1. [Gambaran Umum](#1-gambaran-umum)
2. [Struktur Folder](#2-struktur-folder)
3. [Arsitektur Sistem](#3-arsitektur-sistem)
4. [Alur Topic ROS2](#4-alur-topic-ros2)
5. [Penjelasan Per File](#5-penjelasan-per-file)
6. [State Machine](#6-state-machine)
7. [Cara Build & Jalankan](#7-cara-build--jalankan)
8. [Parameter Tuning](#8-parameter-tuning)
9. [Troubleshooting](#9-troubleshooting)

---

## 1. Gambaran Umum

`KickNRush` adalah package ROS2 Python yang mengatur **logika tingkat tinggi** robot OP3 untuk:

```
Cari Bola → Dekati Bola → Orbit Mengelilingi Bola → Sejajar Gawang → Tendang
```

Package ini **bergantung pada** package `headControl26` yang sudah berjalan terlebih dahulu untuk:
- Deteksi bola via YOLO OpenVINO (`vision.py`)
- Head tracking via PID (`mainHeadControl.py`)

---

## 2. Struktur Folder

```
aroc26/src/KickNRush/
├── package.xml                  ← metadata package ROS2
├── setup.py                     ← entry point & data files
├── setup.cfg                    ← script dir config
├── resource/
│   └── KickNRush                ← marker file (wajib ROS2)
├── launch/
│   └── KickNRush.launch.py      ← launch file utama
└── KickNRush/
    ├── __init__.py
    ├── main.py                  ← entry point node ROS2
    ├── task_control.py          ← state machine utama (otak robot)
    ├── motion_orbit.py          ← logika orbit mengelilingi bola
    ├── motion_crab.py           ← logika crab walk (geser samping)
    └── goal_alignment.py        ← deteksi sejajar gawang (IMU + YOLO)
```

---

## 3. Arsitektur Sistem

```
┌──────────────────────────────────────────────────────────────────┐
│                        headControl26                             │
│                                                                  │
│  ┌─────────────┐    /obj_detect          ┌──────────────────┐   │
│  │  vision.py  │───────────────────────► │ mainHeadControl  │   │
│  │  (YOLO)     │───/obj_detect_goal────► │ (Head PID)       │   │
│  │             │───/obj_detect_ball_bbox  └──────────────────┘   │
│  └─────────────┘                                                 │
└──────────────────────────┬───────────────────────────────────────┘
                           │  /obj_detect
                           │  /obj_detect_goal
                           │  /obj_detect_ball_bbox
                           ▼
┌──────────────────────────────────────────────────────────────────┐
│                         KickNRush                                │
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                    main.py (Node)                        │   │
│  │                   timer 20 Hz                            │   │
│  │                        │                                 │   │
│  │                        ▼                                 │   │
│  │  ┌─────────────────────────────────────────────────┐    │   │
│  │  │              task_control.py                    │    │   │
│  │  │           (State Machine Utama)                 │    │   │
│  │  │                                                 │    │   │
│  │  │  ┌─────────────┐  ┌────────────┐  ┌─────────┐  │    │   │
│  │  │  │motion_orbit │  │motion_crab │  │goal_    │  │    │   │
│  │  │  │             │  │            │  │alignment│  │    │   │
│  │  │  │ hitung      │  │ hitung     │  │         │  │    │   │
│  │  │  │ params      │  │ params     │  │IMU+YOLO │  │    │   │
│  │  │  └─────────────┘  └────────────┘  └─────────┘  │    │   │
│  │  └─────────────────────────────────────────────────┘    │   │
│  └──────────────────────────────────────────────────────────┘   │
└──────────────────────────┬───────────────────────────────────────┘
                           │
              ┌────────────┼────────────┐
              ▼            ▼            ▼
   /robotis/walking/  /robotis/     /robotis/
   set_params         walking/      action/
                      command       page_num
              │            │            │
              ▼            ▼            ▼
┌──────────────────────────────────────────────────────────────────┐
│                      ROBOTIS OP3                                 │
│              walking_module    action_module                     │
└──────────────────────────────────────────────────────────────────┘
```

---

## 4. Alur Topic ROS2

### Topic yang DIPUBLISH oleh `vision.py` (headControl26)

| Topic | Tipe | Format | Keterangan |
|---|---|---|---|
| `/obj_detect` | `String` | `"cx,cy"` | Posisi tengah bola di frame |
| `/obj_detect_ball_bbox` | `String` | `"cx,cy,w,h"` | Bbox lengkap bola (untuk estimasi jarak) |
| `/obj_detect_goal` | `String` | `"cx,cy,w,h"` | Bbox gawang (untuk GoalAlignment) |

### Topic yang DISUBSCRIBE oleh `task_control.py`

| Topic | Tipe | Keterangan |
|---|---|---|
| `/obj_detect` | `String` | Posisi bola |
| `/obj_detect_ball_bbox` | `String` | Bbox bola untuk estimasi jarak |
| `/robotis/open_cr/button` | `String` | Tombol START/MODE OpenCR |
| `/robotis/open_cr/imu` | `Imu` | Data IMU untuk fall detection |

### Topic yang DISUBSCRIBE oleh `goal_alignment.py`

| Topic | Tipe | Keterangan |
|---|---|---|
| `/robotis/open_cr/imu` | `Imu` | Yaw rate untuk estimasi orientasi robot |
| `/obj_detect_goal` | `String` | Posisi visual gawang di frame |

### Topic yang DIPUBLISH oleh `task_control.py`

| Topic | Tipe | Keterangan |
|---|---|---|
| `/robotis/walking/set_params` | `WalkingParam` | Parameter jalan (x, y, angle, period) |
| `/robotis/walking/command` | `String` | `"start"` atau `"stop"` |
| `/robotis/enable_ctrl_module` | `String` | Switch modul aktif |
| `/robotis/action/page_num` | `Int32` | Nomor page action (tendang, bangun) |

### Topic yang DIPUBLISH oleh `goal_alignment.py`

| Topic | Tipe | Keterangan |
|---|---|---|
| `/goal_alignment/status` | `String` | `"aligned"` atau `"not_aligned"` |
| `/goal_alignment/debug` | `String` | Info debug sensor fusion |

---

## 5. Penjelasan Per File

### `main.py` — Entry Point Node ROS2

Node utama yang menginisialisasi `TaskControl` dan menjalankan loop 20 Hz.

```python
node = KickNRushNode()
# di dalamnya:
self.task = TaskControl(self)
self.timer = self.create_timer(1.0 / 20, self._loop)
```

Tidak ada logika di sini — semua didelegasikan ke `TaskControl`.

---

### `task_control.py` — State Machine Utama

Otak robot. Mengatur kapan robot harus melakukan apa berdasarkan state saat ini.

**State yang tersedia:**

| State | Aksi |
|---|---|
| `IDLE` | Diam, tunggu tombol START |
| `SEARCH` | Putar badan cari bola |
| `APPROACH` | Maju mendekati bola |
| `ORBIT` | Orbit mengelilingi bola sampai sejajar gawang |
| `ADJUST` | Crab walk fine-tune posisi kaki |
| `KICK` | Tendang bola |
| `GET_UP` | Bangun setelah jatuh |

**Tombol OpenCR:**
- `start` → masuk ke `SEARCH` dari `IDLE`
- `mode` → stop paksa, kembali ke `IDLE`

---

### `motion_orbit.py` — Logika Orbit

Menghitung parameter walking untuk orbit mengelilingi bola.

**Prinsip:**
- Orbit = `y_move` (geser samping) + `angle_move` (rotasi) secara bersamaan
- Orbit **kanan**: `y = negatif` + `angle = positif`
- Orbit **kiri**: `y = positif` + `angle = negatif`

**Method utama:**

```python
orbit.set_direction("right")         # set arah
x, y, angle_deg, period = orbit.get_params()  # ambil params (angle dalam DERAJAT)
```

> ⚠️ `get_params()` return angle dalam **derajat**. Konversi ke radian dilakukan di `task_control.py` saat publish ke `WalkingParam`.

---

### `motion_crab.py` — Logika Crab Walk

Menghitung parameter walking untuk geser samping **tanpa rotasi**.

**Perbedaan dengan orbit:**

| | Orbit | Crab |
|---|---|---|
| y_move | ✅ | ✅ |
| angle_move | ✅ (berputar) | ❌ (selalu 0) |
| Tujuan | Melingkar bola | Fine-tune posisi kaki |

**Mode kecepatan:**

| Mode | Kecepatan y | Kapan dipakai |
|---|---|---|
| `"normal"` | 0.020 m | Default |
| `"fast"` | 0.040 m | Koreksi besar |
| `"slow"` | 0.010 m | Fine-tune akhir (dipakai di ADJUST) |

**Method spesial:**

```python
# Auto-deteksi arah dari posisi bola di frame
direction = crab.estimate_direction_from_ball(ball_cx, frame_w=640, deadband=40)
# Return: "left", "right", atau None (sudah di tengah)
```

---

### `goal_alignment.py` — Deteksi Sejajar Gawang

Menggunakan **fusion dua sensor** untuk deteksi apakah robot sudah menghadap gawang:

#### Sensor 1: IMU Yaw
- Integrasikan `angular_velocity.z` dari `/robotis/open_cr/imu`
- Referensi yaw = 0 saat `reset_yaw()` dipanggil (awal berdiri)
- Terapkan **low-pass filter** (alpha = 0.15) untuk redam noise

#### Sensor 2: YOLO Gawang
- Subscribe `/obj_detect_goal` dari `vision.py`
- Hitung error posisi gawang dari tengah frame (pixel)
- Timeout 2 detik jika gawang tidak terdeteksi

#### Mode Fusion

| Mode | Kondisi | Bobot |
|---|---|---|
| **DUAL** | IMU + YOLO aktif | IMU 60% + YOLO 40% |
| **IMU only** | YOLO tidak deteksi gawang | IMU 100% |
| **YOLO only** | IMU tidak tersedia | YOLO 100% |

#### Kondisi Sejajar (AND logic)

```
|yaw_error| < 8°   (IMU_TOLERANCE)
AND
|goal_pixel_error| < 60px   (GOAL_PIXEL_TOLERANCE, jika gawang terlihat)
```

Harus terpenuhi selama **5 frame berturut-turut** (ALIGN_CONFIRM_COUNT) sebelum dinyatakan sejajar — untuk menghindari false positive.

---

## 6. State Machine

```
                    ┌─────────────────────┐
                    │        IDLE         │
                    │  tunggu tombol START│
                    └──────────┬──────────┘
                               │ tombol "start"
                               ▼
              ┌────────────────────────────┐
         ┌───►│          SEARCH            │◄──────┐
         │    │  putar badan cari bola     │       │
         │    └───────────┬────────────────┘       │
         │                │ bola terdeteksi         │
         │                ▼                         │
         │    ┌───────────────────────────┐         │
         │    │         APPROACH          │         │
         │    │  maju mendekati bola      │         │
         │    └───────────┬───────────────┘         │
         │                │ bbox bola >= 100px       │
         │                ▼                         │
         │    ┌───────────────────────────┐         │
         │    │          ORBIT            │         │
         │    │  orbit kiri/kanan         │         │
         │    │  (GoalAlignment auto)     │         │
         │    └───────────┬───────────────┘         │
         │                │ is_aligned() = True      │
         │                ▼                         │
         │    ┌───────────────────────────┐         │
         │    │          ADJUST           │         │
         │    │  crab slow kiri/kanan     │         │
         │    └───────────┬───────────────┘         │
         │                │ bola di tengah frame     │
         │                ▼                         │
         │    ┌───────────────────────────┐         │
         │    │          KICK             │         │
         │    │  tendang kanan/kiri       │─────────┘
         │    └───────────────────────────┘
         │
         │    kapan saja: jika robot jatuh
         └────────────────────────────────────────┐
                    ┌───────────────────────────┐  │
                    │         GET_UP            │  │
                    │  bangun depan/belakang    │──┘
                    └───────────────────────────┘
```

---

## 7. Cara Build & Jalankan

### Build

```bash
cd ~/aroc26
colcon build --packages-select KickNRush
source install/setup.bash
```

### Jalankan

```bash
# Terminal 1: jalankan vision + head tracking
ros2 launch headControl26 headControl26.launch.py

# Terminal 2: jalankan KickNRush
ros2 launch KickNRush KickNRush.launch.py

# Atau dengan debug mode:
ros2 launch KickNRush KickNRush.launch.py log_level:=debug
```

### Monitor Topic

```bash
# Lihat status alignment
ros2 topic echo /goal_alignment/status

# Lihat debug alignment (IMU + YOLO error)
ros2 topic echo /goal_alignment/debug

# Lihat deteksi bola
ros2 topic echo /obj_detect

# Lihat deteksi gawang
ros2 topic echo /obj_detect_goal
```

---

## 8. Parameter Tuning

### `motion_orbit.py`

| Konstanta | Default | Keterangan |
|---|---|---|
| `ORBIT_Y_SPEED` | `0.030` m | Kecepatan geser saat orbit. Naikan jika orbit terlalu lambat |
| `ORBIT_ANGLE_SPEED` | `7.0°` | Sudut rotasi per langkah. Naikan jika bola cepat keluar frame |
| `ORBIT_X_FORWARD` | `0.010` m | Komponen maju kecil agar radius tidak membesar |
| `ORBIT_PERIOD` | `0.55` s | Period satu langkah. Turunkan untuk orbit lebih cepat |

### `motion_crab.py`

| Konstanta | Default | Keterangan |
|---|---|---|
| `CRAB_Y_SPEED` | `0.020` m | Kecepatan geser normal |
| `CRAB_Y_FAST` | `0.040` m | Kecepatan geser cepat |
| `CRAB_Y_SLOW` | `0.010` m | Kecepatan geser lambat (dipakai saat ADJUST) |

### `goal_alignment.py`

| Konstanta | Default | Keterangan |
|---|---|---|
| `IMU_TOLERANCE` | `8.0°` | Toleransi sudut IMU. Naikan jika terlalu sering orbit |
| `GOAL_PIXEL_TOLERANCE` | `60` px | Toleransi posisi gawang di frame |
| `GOAL_DETECT_TIMEOUT` | `2.0` s | Timeout gawang tidak terdeteksi sebelum fallback ke IMU |
| `YAW_FILTER_ALPHA` | `0.15` | Low-pass filter IMU. Turunkan untuk lebih smooth |
| `ALIGN_CONFIRM_COUNT` | `5` | Jumlah frame konsisten sebelum dinyatakan aligned |

### `task_control.py`

| Konstanta | Default | Keterangan |
|---|---|---|
| `BALL_WIDTH_FAR` | `50` px | Bbox bola = masih jauh |
| `BALL_WIDTH_CLOSE` | `100` px | Bbox bola = sudah dekat, mulai orbit |
| `BALL_CENTER_DEADBAND` | `40` px | Toleransi posisi bola dari tengah frame sebelum tendang |
| `APPROACH_X` | `0.030` m | Kecepatan maju saat approach |
| `BALL_LOST_TIMEOUT` | `1.5` s | Timeout bola hilang sebelum balik ke SEARCH |
| `ACTION_KICK_RIGHT` | `83` | Page number action tendang kaki kanan |
| `ACTION_KICK_LEFT` | `84` | Page number action tendang kaki kiri |
| `ACTION_GETUP_FRONT` | `122` | Page number action bangun dari jatuh depan |
| `ACTION_GETUP_BACK` | `123` | Page number action bangun dari jatuh belakang |

---

## 9. Troubleshooting

### Robot tidak mau orbit / orbit sangat lambat
- Cek nilai `ORBIT_Y_SPEED` dan `ORBIT_ANGLE_SPEED` di `motion_orbit.py`
- Monitor `/goal_alignment/debug` untuk lihat apakah error alignment terdeteksi

### Robot orbit terus tidak berhenti
- Naikan `IMU_TOLERANCE` di `goal_alignment.py`
- Naikan `GOAL_PIXEL_TOLERANCE` jika gawang susah terdeteksi
- Turunkan `ALIGN_CONFIRM_COUNT` jika terlalu ketat

### Gawang tidak terdeteksi (mode fallback ke IMU)
- Pastikan nama class di model YOLO adalah `"gawang"` (cek `CLASS_GOAL` di `vision.py`)
- Cek `/obj_detect_goal` dengan `ros2 topic echo /obj_detect_goal`
- Pastikan `headControl26` sudah jalan dan kamera aktif

### Robot jatuh tidak bisa bangun
- Cek page number `ACTION_GETUP_FRONT` dan `ACTION_GETUP_BACK` di `task_control.py`
- Sesuaikan dengan page di file motion robot yang dipakai

### Yaw drift (robot melenceng dari arah gawang)
- Turunkan `YAW_FILTER_ALPHA` di `goal_alignment.py` untuk filter lebih agresif
- Panggil `reset_yaw()` lebih sering (sudah dipanggil setelah setiap tendang)
