# Rencana Lokalisasi Robot OP3 — AROC26

Dokumen ini menjelaskan gambaran besar rencana implementasi sistem lokalisasi robot
dengan menggabungkan **Visual Landmark Detection** dan **Pelvis Odometry** (`op3_localization`).

---

## Daftar Isi

1. [Tujuan](#1-tujuan)
2. [Sumber Data yang Tersedia](#2-sumber-data-yang-tersedia)
3. [Metode 1: Visual Landmark Detection](#3-metode-1-visual-landmark-detection)
4. [Metode 2: Pelvis Odometry (op3_localization)](#4-metode-2-pelvis-odometry-op3_localization)
5. [Fusion Kedua Metode](#5-fusion-kedua-metode)
6. [Estimasi Posisi di Lapangan](#6-estimasi-posisi-di-lapangan)
7. [Rencana Implementasi Bertahap](#7-rencana-implementasi-bertahap)

---

## 1. Tujuan

Robot mengetahui **posisi approx (x, y)** dirinya di lapangan secara real-time, sehingga bisa:
- Menghindari keluar lapangan
- Menentukan dari sisi mana harus orbit ke bola
- Memilih strategi approach yang lebih cerdas
- Di masa depan: koordinasi antar robot (multi-agent)

---

## 2. Sumber Data yang Tersedia

### A. Model YOLO (sudah ada di `metadata.yaml`)
```
0: Gawang   → gawang lawan
1: LKANAN   → sudut garis penalti kanan (bentuk L dilihat dari depan gawang)
2: LKIRI    → sudut garis penalti kiri
3: TKANAN   → pertemuan 3 garis terluar kanan (bentuk T)
4: TKIRI    → pertemuan 3 garis terluar kiri
5: bola     → bola
```

### B. `op3_localization` (sudah ada, sudah di-build)
```
Subscribe: /robotis/pelvis_pose       → pose pelvis dari walking module
Publish  : TF "world" → "body_link"  → estimasi posisi robot (odometry)
```
Ini adalah **dead reckoning** — akumulasi pergerakan pelvis dari kinematik walking.
Tidak ada koreksi, jadi drift makin lama.

### C. IMU OpenCR
```
/robotis/open_cr/imu → orientasi (quaternion) + akselerasi
```
Sudah dipakai di `goal_alignment.py` untuk yaw.

---

## 3. Metode 1: Visual Landmark Detection

### Prinsip
Estimasi posisi robot berdasarkan **ukuran dan posisi landmark di frame kamera**.

```
Landmark besar di frame  → robot dekat landmark tersebut
Landmark kecil di frame  → robot jauh dari landmark tersebut
Posisi landmark di frame → robot ada di kiri/kanan/depan landmark
```

### Landmark dan Artinya

```
Tampak atas lapangan:

  TKIRI ──────────────────────── TKANAN
    │                                │
    │    LKIRI ──────── LKANAN       │
    │      │   GAWANG    │           │
    │    LKIRI ──────── LKANAN       │
    │                                │
  TKIRI ──────────────────────── TKANAN
```

| Landmark | Terdeteksi | Estimasi Posisi Robot |
|---|---|---|
| `Gawang` besar | depan kamera | robot dekat gawang |
| `LKANAN` terlihat | kamera menghadap kanan-depan | robot di area penalti kanan |
| `LKIRI` terlihat | kamera menghadap kiri-depan | robot di area penalti kiri |
| `TKANAN` terlihat | kamera menghadap kanan | robot mendekati batas terluar kanan |
| `TKIRI` terlihat | kamera menghadap kiri | robot mendekati batas terluar kiri |

### Estimasi Jarak dari Bbox

```python
# Makin besar bbox → makin dekat
# Konstanta focal length perlu dikalibrasi

jarak_m = (lebar_nyata_m * focal_length_px) / lebar_bbox_px

# Estimasi kasar tanpa kalibrasi (rule-based):
if lebar_bbox > 200px:  zona = "SANGAT_DEKAT"
if lebar_bbox > 100px:  zona = "DEKAT"
if lebar_bbox > 50px:   zona = "SEDANG"
if lebar_bbox < 50px:   zona = "JAUH"
```

### Zona Lapangan (Rule-Based)

```
┌──────────┬──────────┬──────────┐
│  ZONA_TL │  ZONA_T  │  ZONA_TR │
│ T+L kiri │  tengah  │ T+L kanan│
├──────────┼──────────┼──────────┤
│  ZONA_L  │  ZONA_M  │  ZONA_R  │
│  L kiri  │  aman    │  L kanan │
├──────────┼──────────┼──────────┤
│  ZONA_BL │  ZONA_B  │  ZONA_BR │
└──────────┴──────────┴──────────┘
```

**Kelebihan:**
- Tidak butuh kalibrasi kamera
- Implementasi sederhana dan cepat
- Langsung bisa dipakai untuk logika hindari batas lapangan

**Kekurangan:**
- Posisi kasar (hanya zona, bukan koordinat presisi)
- Bergantung pada orientasi kepala robot (kepala harus menghadap landmark)

---

## 4. Metode 2: Pelvis Odometry (`op3_localization`)

### Prinsip
Akumulasi pergerakan pelvis (pinggul) robot dari walking module → estimasi posisi (x, y) di dunia.

```
Setiap langkah robot → walking module hitung delta pose pelvis
op3_localization akumulasikan delta pose → posisi (x, y, yaw) robot
```

### Topic yang Dipakai

```
Subscribe: /robotis/pelvis_pose       → delta pose pelvis per langkah
Subscribe: /robotis/pelvis_pose_reset → reset posisi ke (0, 0)
Publish  : TF "world" → "body_link"  → posisi robot di dunia
```

### Cara Baca Posisi dari TF

```python
from tf2_ros import Buffer, TransformListener

tf_buffer   = Buffer()
tf_listener = TransformListener(tf_buffer, node)

# Baca posisi robot (body_link) relatif terhadap world
transform = tf_buffer.lookup_transform('world', 'body_link', rclpy.time.Time())
x   = transform.transform.translation.x
y   = transform.transform.translation.y
yaw = euler_from_quaternion([...transform.transform.rotation...])[2]
```

### Reset Odometry

```python
# Reset posisi ke (0, 0) — panggil saat robot berdiri di posisi awal
pub_reset.publish(String(data="reset"))
```

**Kelebihan:**
- Posisi dalam koordinat (x, y) nyata dalam meter
- Kontinyu — update setiap langkah
- Sudah tersedia, tinggal dipakai

**Kekurangan:**
- **Drift** — makin lama makin melenceng tanpa koreksi
- Tidak tahu posisi absolut di lapangan — hanya relatif dari titik start
- Akurasi bergantung pada kinematik walking module

---

## 5. Fusion Kedua Metode

### Konsep: EKF Localization (Extended Kalman Filter)

```
┌─────────────────────────────────────────┐
│           PREDIKSI (Odometry)           │
│  op3_localization → posisi (x, y, yaw) │
│  Update setiap langkah (~10-20 Hz)      │
└──────────────────┬──────────────────────┘
                   │
                   ▼ prediksi posisi
┌─────────────────────────────────────────┐
│          KOREKSI (Visual Landmark)      │
│  Deteksi LKANAN/LKIRI/TKANAN/TKIRI     │
│  → estimasi jarak ke landmark           │
│  → koreksi drift odometry              │
│  Update saat landmark terdeteksi (~5 Hz)│
└──────────────────┬──────────────────────┘
                   │
                   ▼ posisi terkoreksi
┌─────────────────────────────────────────┐
│         ESTIMASI POSISI FINAL           │
│  (x, y, yaw) di lapangan               │
│  publish ke /robot_pose                 │
└─────────────────────────────────────────┘
```

### Cara Koreksi Sederhana (tanpa EKF penuh)

Jika tidak mau implementasi EKF penuh, bisa pakai **weighted average**:

```python
# Posisi dari odometry (kontinyu tapi drift)
pos_odom = (x_odom, y_odom)

# Posisi dari landmark (kasar tapi tidak drift)
pos_landmark = estimate_from_landmark(bbox_TKANAN, bbox_LKANAN, ...)

# Fusion berbobot
# Makin lama robot jalan → percaya lebih ke landmark
alpha = min(drift_time / 30.0, 0.8)   # alpha naik seiring waktu berjalan
pos_fused_x = (1 - alpha) * pos_odom[0] + alpha * pos_landmark[0]
pos_fused_y = (1 - alpha) * pos_odom[1] + alpha * pos_landmark[1]
```

### Koordinat Lapangan

```
Titik (0, 0) = tengah lapangan
x positif    = arah gawang lawan
y positif    = sisi kiri lapangan

┌─────────────────────────────────────┐
│ (-4.5, 3)              (4.5, 3)     │
│                                     │
│     (-1.5, 1)  (0,0)  (1.5, 1)     │
│                                     │
│ (-4.5,-3)              (4.5,-3)     │
└─────────────────────────────────────┘
     ↑                        ↑
  TKIRI                    TKANAN
```

---

## 6. Estimasi Posisi di Lapangan

### Dari Landmark Tunggal

```python
# Jika hanya TKANAN terdeteksi:
# → robot di sisi kanan lapangan
# → estimasi y ≈ +2.5 sampai +3.0 m (dekat batas kanan)
# → x tidak diketahui

# Jika hanya Gawang terdeteksi besar:
# → robot dekat gawang lawan
# → estimasi x ≈ +3.0 sampai +4.5 m
# → y tidak diketahui
```

### Dari Dua Landmark (Triangulasi Kasar)

```python
# Jika TKANAN + Gawang terdeteksi bersamaan:
# → robot di pojok kanan dekat gawang
# → estimasi (x, y) ≈ (+3.5, +2.5)

# Jika LKANAN + Gawang terdeteksi:
# → robot di dalam area penalti kanan
# → estimasi (x, y) ≈ (+2.0, +1.0)
```

---

## 7. Rencana Implementasi Bertahap

### Tahap 1 — Deteksi Landmark (Mudah)
- [ ] Tambah deteksi `LKANAN`, `LKIRI`, `TKANAN`, `TKIRI` di `vision.py`
- [ ] Publish masing-masing ke topic terpisah: `/obj_detect_lkanan`, `/obj_detect_tkanan`, dst
- [ ] Format: `"cx,cy,w,h"` sama seperti `/obj_detect_goal`

### Tahap 2 — Rule-Based Zone (Sedang)
- [ ] Buat `localization.py` di package `KickNRush`
- [ ] Subscribe semua topic landmark
- [ ] Estimasi zona (TL, T, TR, L, M, R, BL, B, BR) dari landmark yang terdeteksi
- [ ] Publish `/robot_zone` → String (nama zona)
- [ ] Gunakan di `task_control.py` untuk logika hindari batas lapangan

### Tahap 3 — Odometry Integration (Sedang-Sulit)
- [ ] Subscribe TF `world` → `body_link` dari `op3_localization`
- [ ] Publish `/robot_pose` → `geometry_msgs/PoseStamped` (x, y, yaw)
- [ ] Reset odometry saat robot berdiri di posisi awal
- [ ] Tambah di `task_control.py` untuk keputusan berbasis posisi

### Tahap 4 — Fusion EKF (Sulit)
- [ ] Implementasi EKF sederhana
- [ ] Odometry sebagai model prediksi
- [ ] Landmark sebagai model observasi/koreksi
- [ ] Output: posisi (x, y, yaw) terkoreksi di lapangan
- [ ] Publish `/robot_pose_fused`

### Tahap 5 — Integrasi ke Task Control (Opsional)
- [ ] Robot pilih arah orbit berdasarkan posisi di lapangan
- [ ] Robot hindari keluar lapangan secara otomatis
- [ ] Robot approach bola dari arah yang optimal menuju gawang
