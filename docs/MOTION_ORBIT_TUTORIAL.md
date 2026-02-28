# 🪐 Tutorial Motion Orbit Robot Humanoid (OP3)
> Bahasa bayi edition 🍼 — Berdasarkan kode di `StillUseless`, `KinematrixHumanoid`, dan `ros2iwandwi`

---

## 🧠 Dulu vs Sekarang

| Folder | ROS Version | File Utama |
|---|---|---|
| `StillUseless` | ROS 1 (`rospy`) | `task_control.py`, `motion_module.py` |
| `KinematrixHumanoid` | ROS 1 (`rospy`) | sama seperti StillUseless |
| `ros2iwandwi` | **ROS 2 (`rclpy`)** ← terbaru | `main_task_control.py`, `motion_module.py` |

> ✅ **Gunakan `ros2iwandwi`** untuk project terbaru!

---

## 🍼 Konsep Dasar: Apa Itu Motion Orbit?

**Motion Orbit** = robot humanoid jalan **melingkar mengelilingi bola**, sambil **kepalanya terus menatap bola** dan **badannya selalu menghadap ke bola**.

Ibarat planet 🪐 ngorbit matahari ☀️, tapi planetnya bisa jalan sendiri dan matanya terus ngeliatin mataharinya.

```
          🤖 ← robot jalan melingkar
         /
        /  ← orbit path
       ⚽   ← bola (pusat orbit)
```

### Parameter Walking yang dipakai:
```python
module.Motion_WalkingParams(x, y, o, t)
#  x = maju/mundur (m)     → orbit: kecil saja, atau 0
#  y = geser kiri/kanan    → INI KUNCI ORBIT! (+ = kiri, - = kanan)
#  o = putar badan         → gunakan outBodyPanning dari PID
#  t = periode langkah (s) → biasanya 0.60
```

---

## 🗂️ Struktur File yang Perlu Dipahami

```
ros2iwandwi/src/robotis_manager/robotis_manager/
├── main_task_control.py       ← 🧠 OTAK: state machine robot
├── motion_control_module.py   ← 🏗️ BaseControl: isFalling, isBallFound, HeadScan
├── motion_module.py           ← 🦵 KAKI: Motion_WalkingParams, Motion_Start, dll
├── motion_PID.py              ← 📐 PID: outBodyPanning (untuk putar badan)
├── motion_head_control_module.py ← 👁️ KEPALA: HeadTracking, scan bola
├── motion_data_flow_handler.py   ← 📦 DATA: Data.Head.ballFlag, headPan, dll
└── motion_utility.py          ← ⚙️ UTILITY: Walking params lengkap
```

---

## 🔑 Data Penting yang Kamu Pakai

Semua data robot ada di class `Data` dari `motion_data_flow_handler.py`:

| Data | Artinya | Kapan dipakai |
|---|---|---|
| `Data.Head.ballFlag` | `True` kalau bola ketemu kamera | Syarat utama orbit |
| `Data.Head.headPan` | Posisi kepala kiri-kanan (-1.0 s/d +1.0) | Input PID badan |
| `Data.Head.headTilt` | Posisi kepala atas-bawah | Cek bola deket/jauh |
| `Data.Head.ballCZ` | Ukuran bola di kamera (makin besar = makin deket) | Cek jarak bola |
| `Data.Head.ballCX` | Posisi X bola di frame kamera | Koreksi posisi |
| `Data.Systems.state` | State robot: `'RUN'`, `'STOP'`, dll | State machine |
| `Data.Systems.task` | Sub-task: `'GO'`, `'SCAN'`, `'ORBIT'`, dll | Sub-state machine |
| `Data.Motion.imuData` | Data IMU (accelerometer) | Deteksi jatuh |

---

## 📐 PID yang Sudah Ada (Jangan Dibuat Ulang!)

Di `BaseControl.__init__()` dan `main_task_control.py` loop, sudah ada 3 PID:

| Index | Nama | Fungsi | Output |
|---|---|---|---|
| `BODY_REF (0)` | Gyro reference | Koreksi badan dari gyro | `outBodyReference` |
| `BODY_PAN (1)` | Head pan → badan | **Putar badan supaya nghadap bola** | `outBodyPanning` ← **ini yang dipakai orbit!** |
| `STEP_CORRECTION (2)` | Step correction | Koreksi langkah | `outStepCorrection` |

### Cara kerja `outBodyPanning`:
```python
# Di loop() main_task_control.py (sudah otomatis jalan):
self.pid[BODY_PAN].calculate(Data.Head.headPan)
self.outBodyPanning = self.pid[BODY_PAN].getOutput()
self.outBodyPanning = mapF(self.outBodyPanning, -500, 500, 500, -500)
# → outBodyPanning ini langsung dipakai sebagai parameter 'o' di WalkingParams
```

Artinya: kalau kepala noleh kiri untuk lihat bola → `outBodyPanning` kasih sinyal ke badan untuk putar kiri → badan ikut nghadap bola otomatis! 🤖

---

## 🚶 Tutorial Langkah demi Langkah: Membuat Motion Orbit

### LANGKAH 1 — Pahami State Machine yang Sudah Ada

Buka `main_task_control.py`. Robot punya state seperti ini:

```
PRE_RUN → RUN → [task: GO → SCAN → POSITION → RETRY]
                                 ↑
                         Kamu tambahkan ORBIT di sini!
```

State machine ada di dictionary `system_state_functions`:
```python
self.system_state_functions = {
    'PRE_RUN':     self.SystemPreRun,
    'RUN':         self.SystemRun,
    'STOP':        self.SystemStop,
    # ... dll
}
```

Sub-task ada di dalam `SystemRun()`:
```python
if Data.Systems.task == 'GO':    ...
elif Data.Systems.task == 'SCAN': ...
elif Data.Systems.task == 'POSITION': ...
```

---

### LANGKAH 2 — Tambahkan Konstanta Orbit di `TuningModule`

Buka `main_task_control.py`, di class `TuningModule`, tambahkan:

```python
class TuningModule:
    def __init__(self):
        # ... (yang sudah ada) ...
        self.kfBallSizeToKick            = 30.25
        self.kfBallSizeToSlowDown        = 44.25
        # ... dst ...

        # 🆕 TAMBAHKAN INI untuk orbit:
        self.kfOrbitYSpeed               = 0.025   # kecepatan geser melingkar (m)
        self.kfOrbitXSpeed               = 0.005   # maju sedikit supaya tetap dekat bola
        self.kfOrbitPeriod               = 0.60    # periode langkah orbit
        self.kfOrbitBallSizeMin          = 20.0    # ukuran bola minimum untuk mulai orbit
        self.kfOrbitBallSizeMax          = 50.0    # ukuran bola maksimum (terlalu deket)
        self.kfOrbitDirection            = 1       # 1 = orbit kiri, -1 = orbit kanan
```

> **Penjelasan bayi 🍼**: `kfOrbitYSpeed` itu kayak ngatur seberapa cepet robot geser melingkar. Mulai dari 0.025 dulu, nanti dicoba-coba.

---

### LANGKAH 3 — Buat Fungsi `SystemOrbit()` di Class `TaskControl`

Di `main_task_control.py`, di dalam class `TaskControl`, tambahkan method baru:

```python
def SystemOrbit(self):
    """
    🪐 Motion Orbit: robot mengelilingi bola sambil terus menghadap bola.
    
    Prinsip:
    - y_move = geser melingkar (kiri atau kanan)
    - o (angle) = outBodyPanning dari PID supaya badan selalu nghadap bola
    - kepala tetap tracking bola (headEnable = True)
    """
    if not self.isBallFound():
        # Bola hilang saat orbit → balik ke SCAN
        self.module.Motion_WalkingParams(0.00, 0.00, self.outBodyReference, self.kfOrbitPeriod)
        self.module.Motion_Start()
        if Ticks() - self.taskTimer >= 3000:
            Data.Systems.task = 'SCAN'
            self.taskTimer = Ticks()
        return

    # ✅ Bola ketemu → jalankan orbit
    y_orbit = self.kfOrbitYSpeed * self.kfOrbitDirection  # geser melingkar
    x_orbit = self.kfOrbitXSpeed                          # maju dikit biar tetap deket

    # Kalau bola terlalu deket → mundur
    if self.ballSizeFiltered > self.kfOrbitBallSizeMax:
        x_orbit = -self.kfOrbitXSpeed

    # Kalau bola terlalu jauh → maju lebih
    elif self.ballSizeFiltered < self.kfOrbitBallSizeMin:
        x_orbit = self.kfOrbitXSpeed * 2

    # 🔑 Kunci orbit: outBodyPanning bikin badan terus nghadap bola!
    self.module.Motion_WalkingParams(x_orbit, y_orbit, self.outBodyPanning, self.kfOrbitPeriod)
    self.module.Motion_Start()
```

> **Penjelasan bayi 🍼**:
> - `y_orbit` = robot geser ke samping (kayak kepiting 🦀 jalan)
> - `outBodyPanning` = PID yang otomatis ngoreksiin badan supaya tetep nghadap bola
> - Kalau bola ilang, robot berhenti orbit dan scan lagi

---

### LANGKAH 4 — Tambahkan Sub-Task `'ORBIT'` di `SystemRun()`

Di method `SystemRun()`, tambahkan kondisi orbit:

```python
def SystemRun(self):
    # ... (kode filtering yang sudah ada, jangan dihapus!) ...
    self.ballSizeFiltered  = (self.ballSizeFiltered * KF_BALL_SIZE) + Data.Head.ballCZ
    self.ballSizeFiltered /= KF_BALL_SIZE + 1
    self.ballPosFiltered   = (self.ballPosFiltered * KF_BALL_POS) + Data.Head.ballCXFiltered
    self.ballPosFiltered  /= KF_BALL_POS + 1

    if Data.Systems.task == 'GO':
        # ... (kode GO yang sudah ada) ...

    elif Data.Systems.task == 'SCAN':
        # ... (kode SCAN yang sudah ada) ...

    elif Data.Systems.task == 'POSITION':
        # ... (kode POSITION yang sudah ada) ...

    elif Data.Systems.task == 'RETRY':
        # ... (kode RETRY yang sudah ada) ...

    # 🆕 TAMBAHKAN INI:
    elif Data.Systems.task == 'ORBIT':
        self.SystemOrbit()
```

---

### LANGKAH 5 — Tentukan Kapan Masuk State Orbit

Ada dua pilihan: **manual (tombol)** atau **otomatis (dari kondisi)**. Pilih salah satu atau keduanya.

#### Pilihan A — Otomatis dari kondisi di `SystemRun()`

Misalnya: setelah `SCAN` dan bola ketemu di jarak tertentu, masuk orbit:

```python
elif Data.Systems.task == 'SCAN':
    if self.isBallFound():
        if abs(self.pid[BODY_PAN].getError()) > self.kfBodyPanCorrection:
            # badan belum nghadap bola → koreksi dulu
            self.module.Motion_WalkingParams(0.00, 0.00, self.outBodyPanning, 0.60)
            self.module.Motion_Start()
        else:
            # 🆕 Kalau bola di jarak orbit yang pas → masuk ORBIT
            if self.kfOrbitBallSizeMin <= self.ballSizeFiltered <= self.kfOrbitBallSizeMax:
                Data.Systems.task = 'ORBIT'
                self.taskTimer = Ticks()
            elif self.ballSizeFiltered < self.kfBallSizeToKick:
                Data.Systems.task = 'POSITION'
                self.taskTimer    = Ticks()
            else:
                self.module.Motion_WalkingParams(self.kfStepSpeed, 0.0, self.outBodyPanning, self.kfStepPeriod)
                self.module.Motion_Start()
```

#### Pilihan B — Manual via SSH command

Di `main_ssh_control.py` atau di tempat SSH command diterima, tambahkan:
```python
if command == 'orbit':
    Data.Systems.task = 'ORBIT'
```

---

### LANGKAH 6 — Pilih Arah Orbit (Kiri atau Kanan)

Di `main_task_control.py` atau bisa diatur dinamis:

```python
# Di TuningModule atau sebelum masuk orbit:
self.kfOrbitDirection = 1    # orbit ke KIRI  (y positif)
self.kfOrbitDirection = -1   # orbit ke KANAN (y negatif)
```

Atau otomatis pilih arah berdasarkan posisi kepala:
```python
# Masuk orbit: pilih arah dari mana bola terakhir dilihat
if Data.Head.headPan > 0:
    self.kfOrbitDirection = 1   # bola di kiri → orbit ke kiri
else:
    self.kfOrbitDirection = -1  # bola di kanan → orbit ke kanan
Data.Systems.task = 'ORBIT'
self.taskTimer = Ticks()
```

---

### LANGKAH 7 — Tuning Parameter Orbit

Setelah kode jalan, tuning nilai-nilai ini di `TuningModule`:

| Parameter | Default | Efek kalau dinaikkan | Efek kalau diturunkan |
|---|---|---|---|
| `kfOrbitYSpeed` | `0.025` | Orbit lebih cepat | Orbit lebih pelan |
| `kfOrbitXSpeed` | `0.005` | Maju lebih ke bola | Tetap di tempat |
| `kfOrbitPeriod` | `0.60` | Langkah lebih lambat | Langkah lebih cepat |
| `kfOrbitBallSizeMin` | `20.0` | Orbit dimulai saat bola lebih jauh | Orbit dimulai saat bola lebih deket |
| `kfOrbitBallSizeMax` | `50.0` | Bisa lebih deket ke bola | Orbit lebih jauh dari bola |

> **Tips tuning 🍼**: Coba satu-satu. Pertama pastikan `kfOrbitYSpeed` bisa bikin robot geser. Lalu pastikan `outBodyPanning` (PID) bisa bikin badan selalu nghadap bola. Baru tuning yang lain.

---

## 🔄 Alur Lengkap Motion Orbit (Ringkasan)

```
START
  ↓
PRE_RUN → inisialisasi walking, head tracking aktif
  ↓
RUN / task: GO
  ↓ (bola ketemu, jarak pas)
RUN / task: SCAN
  ↓ (badan sudah lurus, ballSize di range orbit)
RUN / task: ORBIT ← 🪐 DI SINI ORBIT TERJADI
  │
  ├─ bola ketemu → Motion_WalkingParams(x_orbit, y_orbit, outBodyPanning, period)
  │                                               ↑           ↑
  │                                        geser melingkar  badan nghadap bola (PID)
  │
  └─ bola hilang > 3 detik → kembali ke SCAN
```

---

## 📋 Checklist Implementasi

- [ ] **Step 1**: Pahami state machine di `main_task_control.py`
- [ ] **Step 2**: Tambah konstanta orbit di `TuningModule.__init__()`
- [ ] **Step 3**: Buat method `SystemOrbit()` di class `TaskControl`
- [ ] **Step 4**: Tambah `elif Data.Systems.task == 'ORBIT': self.SystemOrbit()` di `SystemRun()`
- [ ] **Step 5**: Tentukan kondisi masuk orbit (otomatis dari SCAN atau manual)
- [ ] **Step 6**: Pilih arah orbit (kiri/kanan)
- [ ] **Step 7**: Jalankan dan tuning parameter

---

## 🐛 Troubleshooting

| Masalah | Kemungkinan Penyebab | Solusi |
|---|---|---|
| Robot tidak geser | `kfOrbitYSpeed` terlalu kecil | Naikkkan jadi `0.04` |
| Badan tidak nghadap bola | `outBodyPanning` tidak ter-update | Pastikan PID loop jalan (ada di `loop()`) |
| Robot terlalu cepat mendekat | `kfOrbitXSpeed` terlalu besar | Kurangi atau set ke `0.0` |
| Orbit langsung berhenti | `ballSizeFiltered` di luar range | Sesuaikan `kfOrbitBallSizeMin/Max` |
| Robot jatuh saat orbit | `kfOrbitPeriod` terlalu kecil | Naikkkan ke `0.65` atau `0.70` |
| Kepala tidak tracking | `headEnable` tidak `True` | Pastikan `module.headEnable(True)` di `SystemPreRun` |

---

## 📁 File yang Perlu Diedit

1. **`ros2iwandwi/src/robotis_manager/robotis_manager/main_task_control.py`**
   - Tambah konstanta di `TuningModule`
   - Tambah method `SystemOrbit()` di `TaskControl`
   - Tambah kondisi `'ORBIT'` di `SystemRun()`
   - Tambah trigger masuk orbit di `'SCAN'` atau tempat lain

> File lain (`motion_module.py`, `motion_PID.py`, `motion_control_module.py`, dll) **tidak perlu diedit** — semua sudah ready!

---

*Generated from: `StillUseless`, `KinematrixHumanoid (1)`, `ros2iwandwi` — 2026-02-28*
