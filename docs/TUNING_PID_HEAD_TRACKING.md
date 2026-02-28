# 🎛️ Tuning PID Head Tracking — AROC26
> File: `aroc26/src/headControl26/headControl26/mainHeadControl.py`

---

## 🍼 Apa itu Kp, Ki, Kd, Ti, Td?

```
Kp = "seberapa keras reaksi pertama"
     → naik: kepala gerak cepat tapi bisa goyang-goyang (overshoot)
     → turun: kepala lambat tapi stabil

Ki = "ingatan kesalahan dari masa lalu"
     → naik: kepala lebih presisi tapi bisa ngebuang-buang (windup)
     → 0: kepala tidak ingat kesalahan lama (aman untuk awal tuning)

Kd = "rem sebelum sampai target"
     → naik: lebih smooth, kurang goyang
     → 0: tidak ada rem, lebih cepat tapi bisa overshoot

Ti = "seberapa sering Ki di-update" (ms)
     → ini BUKAN gain! ini periode update integral
     → makin kecil = integral lebih sering diupdate
     → makin besar = integral lebih jarang diupdate (lebih lambat)

Td = "seberapa sering Kd di-update" (ms)
     → ini BUKAN gain! ini periode update derivative
     → makin kecil = derivative lebih sering diupdate
     → makin besar = derivative lebih jarang diupdate
```

---

## 🔍 Penjelasan Ti dan Td di Kode

Lihat `motionPID.py` bagian `calculate()`:

```python
# Ti → mengontrol seberapa sering sumError (integral) di-update
if self.currTime - self.lastCurrTime_Ti > self.Ti:
    self.lastCurrTime_Ti = self.currTime
    self.sumError += self.error   # ← integral baru diupdate setiap Ti ms

# Td → mengontrol seberapa sering prevError (derivative) di-update
if self.currTime - self.lastCurrTime_Td > self.Td:
    self.lastCurrTime_Td = self.currTime
    self.prevError = self.error   # ← derivative baru diupdate setiap Td ms
```

Jadi Ti dan Td adalah **timer interval dalam milidetik**, bukan gain!

---

## ❓ Perlu Dituning Tidak?

**Jawaban singkat: TIDAK perlu dituning untuk kasus head tracking ini.**

Alasannya:

| Kondisi | Kenapa Ti/Td tidak perlu dituning |
|---|---|
| Loop berjalan 20Hz = 50ms | Ti=10ms < 50ms → integral **selalu** diupdate setiap loop |
| Ki PAN = 0.0 | Integral tidak aktif → Ti tidak berpengaruh sama sekali |
| Kd PAN = 0.0 | Derivative tidak aktif → Td tidak berpengaruh sama sekali |
| Ki TILT = 0.8 | Ti=10ms < 50ms → integral selalu update, tidak perlu diubah |
| Kd TILT = 1.2 | Td=10ms < 50ms → derivative selalu update, tidak perlu diubah |

**Kesimpulan:**
- Selama `Ti < periode_loop (50ms)` → Ti tidak berpengaruh, biarkan saja
- Selama `Td < periode_loop (50ms)` → Td tidak berpengaruh, biarkan saja
- Nilai default `Ti = Td = 10ms` sudah aman → **tidak perlu diubah**

> ⚠️ Kalau Ti atau Td dinaikkan > 50ms, integral/derivative jadi lebih jarang
> diupdate dari loop → efek PID berkurang. Hindari ini!

---

## 📋 Langkah Tuning yang Benar (Fokus ke Kp, Ki, Kd saja)

### Step 1 — Mulai dari nol semua
```python
PAN_KP  = 0.0
PAN_KI  = 0.0
PAN_KD  = 0.0
TILT_KP = 0.0
TILT_KI = 0.0
TILT_KD = 0.0
```

### Step 2 — Tuning PAN dulu (kiri-kanan)
Naikkan `PAN_KP` pelan-pelan sampai kepala **mulai goyang terus** (osilasi):
```python
PAN_KP = 0.1   # coba → terlalu lambat?
PAN_KP = 0.3   # naikkan
PAN_KP = 0.5   # naikkan lagi
PAN_KP = 0.75  # mulai goyang? → ini Ku (Ultimate Gain)
```
Set `PAN_KP = 0.5 × Ku`:
```python
PAN_KP = 0.375  # = 0.5 × 0.75
```

### Step 3 — Tambah PAN_Kd (kalau masih overshoot)
```python
PAN_KD = 0.05   # coba
PAN_KD = 0.10   # naikkan kalau masih overshoot
```

### Step 4 — Tambah PAN_Ki (kalau kepala tidak tepat di tengah)
```python
PAN_KI = 0.01   # mulai kecil!
PAN_KI = 0.05   # naikkan kalau masih ada steady-state error
```

### Step 5 — Ulangi untuk TILT (atas-bawah)
Sama seperti PAN tapi biasanya `TILT_KP` perlu lebih besar karena
gravitasi melawan gerakan kepala ke atas.

---

## 🔍 Monitor Real-time saat Tuning

```bash
# Posisi bola dari EKF (input PID) — pantau ini saat tuning
ros2 topic echo /vision/ball_ekf

# Posisi servo kepala yang dikirim (output PID)
ros2 topic echo /robotis/head_control/set_joint_states

# Posisi bola raw dari YOLO (sebelum EKF)
ros2 topic echo /vision/ball_measurement
```

---

## 🎯 Gejala & Solusi

| Gejala | Penyebab | Solusi |
|---|---|---|
| Kepala goyang kiri-kanan terus | `PAN_KP` terlalu besar | Turunkan `PAN_KP` |
| Kepala lambat ke bola | `PAN_KP` terlalu kecil | Naikan `PAN_KP` |
| Kepala tidak tepat di tengah | Perlu `Ki` | Tambah `PAN_KI = 0.01` |
| Kepala overshoot lalu balik | Perlu `Kd` | Tambah `PAN_KD = 0.05` |
| Kepala goyang saat sudah di bola | `PAN_KD` terlalu besar | Turunkan `PAN_KD` |
| Kepala lambat noleh ke atas/bawah | `TILT_KP` kecil | Naikan `TILT_KP` |
| Kepala goyang naik-turun | `TILT_KP` terlalu besar | Turunkan `TILT_KP` |
| Kepala masih goyang kecil di bola | Deadband terlalu kecil | Naikan `DEADBAND_X/Y` |

---

## ⚡ Parameter Lain yang Bisa Dituning

### DEADBAND (bukan PID, tapi penting!)
```python
DEADBAND_X = 8   # px — zona mati horizontal
DEADBAND_Y = 8   # px — zona mati vertikal
```
Kalau kepala goyang kecil saat sudah di bola → **naikan ke 15-20px**.
Kalau kepala tidak mau tepat di tengah → **turunkan ke 5px**.

### SCAN_STEP
```python
SCAN_STEP = 0.05  # rad per tick (20Hz) = ~1 rad/s
```
Kalau scan terlalu cepat/lambat saat cari bola → ubah nilai ini.

### EKF Noise (di `ballEKF.py`)
```python
process_noise_pos = 2.0   # noise model posisi  → naik: EKF lebih percaya measurement
process_noise_vel = 8.0   # noise model kecepatan
measure_noise     = 6.0   # noise sensor YOLO   → naik: EKF lebih smooth tapi lambat
```
Kalau kepala terlalu goyang ikuti noise YOLO → **naikan `measure_noise`**.
Kalau kepala terlalu lambat ikuti bola → **turunkan `measure_noise`**.

---

## 📊 Nilai Default Saat Ini

```python
# PAN (kiri-kanan)
PAN_KP  = 0.75   # ← fokus tuning ini dulu
PAN_KI  = 0.0    # ← tambah kalau perlu presisi
PAN_KD  = 0.0    # ← tambah kalau overshoot
PAN_TI  = 10.0   # ms ← TIDAK PERLU DITUNING
PAN_TD  = 10.0   # ms ← TIDAK PERLU DITUNING

# TILT (atas-bawah)
TILT_KP = 1.2    # ← fokus tuning ini dulu
TILT_KI = 0.8    # ← sudah ada, monitor windup
TILT_KD = 1.2    # ← sudah ada, monitor oscillasi
TILT_TI = 10.0   # ms ← TIDAK PERLU DITUNING
TILT_TD = 10.0   # ms ← TIDAK PERLU DITUNING
```

---

*Generated: 2026-02-28 | aroc26/src/headControl26/headControl26/mainHeadControl.py*
