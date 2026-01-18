# 🤖 Panduan Troubleshooting Balance Robot OP3

## Daftar Skenario Masalah

| No | Masalah | Link |
|----|---------|------|
| 1 | [Robot Jatuh ke Depan](#1-robot-jatuh-ke-depan) |
| 2 | [Robot Jatuh ke Belakang](#2-robot-jatuh-ke-belakang) |
| 3 | [Robot Jatuh ke Samping (Kiri/Kanan)](#3-robot-jatuh-ke-samping) |
| 4 | [Robot Goyang/Osilasi saat Berdiri](#4-robot-goyingosilasi-saat-berdiri) |
| 5 | [Robot Jatuh saat Menendang](#5-robot-jatuh-saat-menendang) |
| 6 | [Robot Jatuh saat Berbelok](#6-robot-jatuh-saat-berbelok) |
| 7 | [Robot Jatuh saat Jalan Cepat](#7-robot-jatuh-saat-jalan-cepat) |
| 8 | [Robot Tidak Stabil di Lantai Licin](#8-robot-tidak-stabil-di-lantai-licin) |
| 9 | [Robot Tidak Stabil di Karpet/Rumput](#9-robot-tidak-stabil-di-karpetrumput) |
| 10 | [Robot Lambat Bereaksi terhadap Gangguan](#10-robot-lambat-bereaksi) |

---

## File Konfigurasi

```
aroc26_core/src/ROBOTIS-OP3/
├── op3_walking_module/config/
│   └── param.yaml                    ← Walking parameters + balance gain
│
└── op3_online_walking_module/config/
    ├── balance_gain.yaml             ← Balance control (default)
    ├── balance_gain_conservative.yaml ← Untuk testing/lantai licin
    └── balance_gain_aggressive.yaml   ← Untuk kompetisi/karpet
```

---

## 1. Robot Jatuh ke Depan

### Gejala
- Robot jatuh terjungkal ke depan saat jalan
- Robot menunduk terlalu dalam

### Penyebab
- `foot_pitch_gyro_p_gain` terlalu rendah
- `balance_ankle_pitch_gain` terlalu rendah
- `hip_pitch_offset` tidak sesuai

### Solusi

**Edit `balance_gain.yaml`:**
```yaml
# Naikkan nilai ini
foot_pitch_gyro_p_gain : 0.20   # dari 0.15 → 0.20
foot_pitch_angle_p_gain : 0.30  # dari 0.25 → 0.30
balance_ankle_pitch_gain : 1.00 # dari 0.90 → 1.00
```

**Edit `param.yaml`:**
```yaml
# Kurangi hip pitch offset (robot lebih tegak)
hip_pitch_offset: 3.0   # dari 5.0 → 3.0
```

---

## 2. Robot Jatuh ke Belakang

### Gejala
- Robot jatuh ke belakang saat jalan
- Robot terlalu tegak/mendongak

### Penyebab
- `foot_pitch_angle_p_gain` terlalu rendah
- `hip_pitch_offset` terlalu kecil

### Solusi

**Edit `balance_gain.yaml`:**
```yaml
foot_pitch_angle_p_gain : 0.35  # naikkan
foot_pitch_angle_d_gain : 0.15  # tambah damping
```

**Edit `param.yaml`:**
```yaml
hip_pitch_offset: 7.0   # naikkan (robot lebih condong ke depan)
```

---

## 3. Robot Jatuh ke Samping

### Gejala
- Robot jatuh ke kiri atau kanan
- Robot oleng saat single support phase

### Penyebab
- `foot_roll_gyro_p_gain` terlalu rendah
- `balance_ankle_roll_gain` terlalu rendah
- `balance_hip_roll_gain` terlalu rendah

### Solusi

**Edit `balance_gain.yaml`:**
```yaml
# Naikkan semua roll gain
foot_roll_gyro_p_gain : 0.35    # dari 0.25 → 0.35
foot_roll_angle_p_gain : 0.45   # dari 0.35 → 0.45
balance_hip_roll_gain : 0.45    # dari 0.35 → 0.45
balance_ankle_roll_gain : 0.85  # dari 0.70 → 0.85
```

**Edit `param.yaml`:**
```yaml
balance_hip_roll_gain: 0.45
balance_ankle_roll_gain: 0.85
```

---

## 4. Robot Goyang/Osilasi saat Berdiri

### Gejala
- Robot gemetar/bergetar saat diam
- Kaki robot bergerak-gerak sendiri
- Robot "nervous"

### Penyebab
- Gain terlalu tinggi
- Tidak ada damping (d_gain = 0)
- Cut-off frequency terlalu tinggi (noise)

### Solusi

**Edit `balance_gain.yaml`:**
```yaml
# Turunkan p_gain
foot_roll_gyro_p_gain : 0.15    # turunkan
foot_pitch_gyro_p_gain : 0.10   # turunkan

# Tambah damping
foot_roll_angle_d_gain : 0.20   # dari 0.10 → 0.20
foot_pitch_angle_d_gain : 0.20  # dari 0.10 → 0.20

# Turunkan frequency (filter noise)
roll_gyro_cut_off_frequency : 25.0   # dari 40 → 25
pitch_gyro_cut_off_frequency : 25.0  # dari 40 → 25
```

**Atau gunakan config conservative:**
```bash
cd ~/aroc26_core/src/ROBOTIS-OP3/op3_online_walking_module/config/
cp balance_gain_conservative.yaml balance_gain.yaml
```

---

## 5. Robot Jatuh saat Menendang

### Gejala
- Robot jatuh saat mengangkat kaki untuk tendang
- Robot tidak stabil saat single-leg stance

### Penyebab
- Balance gain tidak cukup untuk single support
- Kecepatan tendang terlalu cepat

### Solusi

**Edit `balance_gain.yaml`:**
```yaml
# Maksimalkan ankle gain (paling penting untuk single leg)
balance_ankle_roll_gain : 0.95
balance_ankle_pitch_gain : 1.00

# Naikkan hip gain
balance_hip_roll_gain : 0.50
balance_knee_gain : 0.45
```

**Tips tambahan:**
- Kurangi kecepatan tendang di motion editor
- Pastikan posisi kaki penumpu lebih lebar

---

## 6. Robot Jatuh saat Berbelok

### Gejala
- Robot jatuh saat turning/berputar
- Robot tidak stabil saat rotasi

### Penyebab
- Rotasi terlalu cepat
- Roll compensation tidak cukup

### Solusi

**Edit `param.yaml`:**
```yaml
# Kurangi kecepatan rotasi maksimum
# Edit di ball_follower.cpp:
# MAX_RL_TURN dari 15.0 → 10.0 derajat
```

**Edit `balance_gain.yaml`:**
```yaml
# Naikkan roll gain
foot_roll_gyro_p_gain : 0.35
balance_ankle_roll_gain : 0.85
```

---

## 7. Robot Jatuh saat Jalan Cepat

### Gejala
- Robot stabil saat jalan pelan, jatuh saat cepat
- Robot "tersandung" langkahnya sendiri

### Penyebab
- Step terlalu besar
- Period time terlalu pendek
- Balance tidak cukup responsif

### Solusi

**Edit `param.yaml`:**
```yaml
# Perpanjang period (jalan lebih pelan tapi stabil)
period_time: 700   # dari 650 → 700 ms

# Kurangi langkah maksimum
# Edit di ball_follower.cpp:
# MAX_FB_STEP dari 40.0 → 30.0 mm
```

**Edit `balance_gain.yaml`:**
```yaml
# Naikkan cut-off frequency (responsif lebih cepat)
roll_gyro_cut_off_frequency : 50.0
pitch_gyro_cut_off_frequency : 50.0
```

---

## 8. Robot Tidak Stabil di Lantai Licin

### Gejala
- Robot slip/tergelincir
- Kaki tidak grip dengan baik

### Solusi

**Gunakan config conservative:**
```bash
cp balance_gain_conservative.yaml balance_gain.yaml
```

**Edit `param.yaml`:**
```yaml
# Kurangi kecepatan
period_time: 750
foot_height: 0.04    # dari 0.06 → 0.04 (angkat kaki lebih rendah)

# Kurangi langkah
step_forward_back_ratio: 0.20  # dari 0.28 → 0.20
```

**Tips hardware:**
- Ganti sol kaki dengan bahan grip lebih baik
- Bersihkan sol kaki dari debu

---

## 9. Robot Tidak Stabil di Karpet/Rumput

### Gejala
- Robot tersangkut/tertahan
- Kaki tidak terangkat cukup tinggi

### Solusi

**Gunakan config aggressive:**
```bash
cp balance_gain_aggressive.yaml balance_gain.yaml
```

**Edit `param.yaml`:**
```yaml
# Naikkan angkatan kaki
foot_height: 0.08    # dari 0.06 → 0.08

# Naikkan balance
balance_hip_roll_gain: 0.45
balance_ankle_pitch_gain: 1.00
```

---

## 10. Robot Lambat Bereaksi

### Gejala
- Robot jatuh karena terlalu lambat bereaksi
- Robot tidak segera koreksi saat didorong

### Penyebab
- Cut-off frequency terlalu rendah
- P_gain terlalu rendah

### Solusi

**Edit `balance_gain.yaml`:**
```yaml
# Naikkan semua p_gain
foot_roll_gyro_p_gain : 0.35
foot_pitch_gyro_p_gain : 0.25
foot_roll_angle_p_gain : 0.45
foot_pitch_angle_p_gain : 0.35

# Naikkan cut-off frequency
roll_gyro_cut_off_frequency : 50.0
pitch_gyro_cut_off_frequency : 50.0
roll_angle_cut_off_frequency : 50.0
pitch_angle_cut_off_frequency : 50.0
```

---

## Quick Reference: Parameter Ranges

| Parameter | Min | Default | Max | Efek |
|-----------|-----|---------|-----|------|
| `foot_roll_gyro_p_gain` | 0.1 | 0.25 | 0.5 | Koreksi roll cepat |
| `foot_pitch_gyro_p_gain` | 0.1 | 0.15 | 0.4 | Koreksi pitch cepat |
| `foot_roll_angle_p_gain` | 0.2 | 0.35 | 0.6 | Koreksi roll posisi |
| `foot_pitch_angle_p_gain` | 0.15 | 0.25 | 0.5 | Koreksi pitch posisi |
| `balance_hip_roll_gain` | 0.2 | 0.35 | 0.5 | Stabilitas hip |
| `balance_knee_gain` | 0.2 | 0.30 | 0.45 | Stabilitas lutut |
| `balance_ankle_roll_gain` | 0.5 | 0.70 | 0.95 | Stabilitas ankle roll |
| `balance_ankle_pitch_gain` | 0.7 | 0.90 | 1.0 | Stabilitas ankle pitch |
| `cut_off_frequency` | 20 | 40 | 60 | Filter noise (Hz) |
| `d_gain` | 0.0 | 0.1 | 0.25 | Damping (anti osilasi) |

---

## Prosedur Tuning di Lapangan

### 1. Persiapan
```bash
# Backup config saat ini
cd ~/aroc26_core/src/ROBOTIS-OP3/op3_online_walking_module/config/
cp balance_gain.yaml balance_gain_backup_$(date +%Y%m%d).yaml
```

### 2. Test Berdiri
- Nyalakan robot, berdiri
- Dorong pelan dari depan/belakang/samping
- Catat reaksi robot

### 3. Test Jalan
- Jalankan walking module
- Perhatikan stabilitas
- Catat masalah yang muncul

### 4. Adjust Parameter
- Sesuaikan parameter berdasarkan tabel di atas
- **Ubah satu parameter pada satu waktu!**
- Test lagi setelah setiap perubahan

### 5. Test Tendang
- Test gerakan tendang
- Pastikan tidak jatuh saat single-leg

### 6. Simpan Config
```bash
# Setelah menemukan config optimal
cp balance_gain.yaml balance_gain_venue_$(date +%Y%m%d).yaml
```

---

## Checklist Kompetisi

- [ ] Test robot di venue sebelum pertandingan
- [ ] Siapkan 3 config (conservative, default, aggressive)
- [ ] Catat kondisi lapangan (licin/karpet/rumput)
- [ ] Backup semua config yang berhasil
- [ ] Siapkan tool untuk edit config cepat

---

*AROC26 Team - 2026*
*Last updated: Januari 2026*
