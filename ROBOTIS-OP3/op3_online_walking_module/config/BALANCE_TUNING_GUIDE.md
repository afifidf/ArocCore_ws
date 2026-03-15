# 🤖 Panduan Tuning Balance Gain OP3

## File Konfigurasi

| File | Penggunaan |
|------|------------|
| `balance_gain.yaml` | **DEFAULT** - Nilai seimbang untuk kebanyakan kondisi |
| `balance_gain_conservative.yaml` | Untuk lantai licin, testing awal |
| `balance_gain_aggressive.yaml` | Untuk kompetisi, lantai karpet/rumput |

## Cara Mengganti Config

```bash
# Backup config saat ini
cp balance_gain.yaml balance_gain_backup.yaml

# Gunakan config conservative
cp balance_gain_conservative.yaml balance_gain.yaml

# Gunakan config aggressive
cp balance_gain_aggressive.yaml balance_gain.yaml
```

## Parameter Utama

### 1. Gyro Gain (Reaksi Cepat)
```yaml
foot_roll_gyro_p_gain   # Koreksi goyang kiri-kanan
foot_pitch_gyro_p_gain  # Koreksi goyang depan-belakang
```
- **Terlalu rendah**: Robot lambat bereaksi, mudah jatuh
- **Terlalu tinggi**: Robot "nervous", osilasi

### 2. IMU Angle Gain (Posisi Absolut)
```yaml
foot_roll_angle_p_gain   # Koreksi posisi roll
foot_pitch_angle_p_gain  # Koreksi posisi pitch
```
- Bekerja sama dengan gyro gain
- Memberikan koreksi berdasarkan orientasi robot

### 3. Joint Balance Gain
```yaml
balance_hip_roll_gain     # 0.2 - 0.5
balance_knee_gain         # 0.2 - 0.4
balance_ankle_roll_gain   # 0.5 - 0.9
balance_ankle_pitch_gain  # 0.7 - 1.0
```
- Ankle paling penting untuk balance!

## Troubleshooting

| Masalah | Solusi |
|---------|--------|
| Robot goyang ke samping | ↑ `foot_roll_gyro_p_gain`, `balance_ankle_roll_gain` |
| Robot jatuh ke depan | ↑ `foot_pitch_gyro_p_gain`, `balance_ankle_pitch_gain` |
| Robot jatuh ke belakang | ↑ `foot_pitch_angle_p_gain` |
| Robot osilasi/gemetar | ↓ semua p_gain, ↑ d_gain |
| Robot bereaksi lambat | ↑ cut_off_frequency (30→50 Hz) |

## Prosedur Tuning

1. **Mulai dari conservative config**
2. **Test berdiri diam** - dorong pelan robot dari samping/depan
3. **Jika stabil**, naikkan gain 10-20%
4. **Jika osilasi**, turunkan gain atau tambah d_gain
5. **Test jalan** - lihat apakah stabil saat walking
6. **Test tendang** - balance saat single-leg stance

## Nilai Referensi per Kondisi

### Indoor (Lantai Keras/Licin)
```yaml
foot_roll_gyro_p_gain : 0.15
foot_pitch_gyro_p_gain : 0.10
balance_ankle_roll_gain : 0.50
balance_ankle_pitch_gain : 0.70
```

### Indoor (Karpet)
```yaml
foot_roll_gyro_p_gain : 0.25
foot_pitch_gyro_p_gain : 0.15
balance_ankle_roll_gain : 0.70
balance_ankle_pitch_gain : 0.90
```

### Outdoor (Rumput Sintetis)
```yaml
foot_roll_gyro_p_gain : 0.35
foot_pitch_gyro_p_gain : 0.25
balance_ankle_roll_gain : 0.85
balance_ankle_pitch_gain : 1.00
```

## Tips Kompetisi

1. **Bawa 3 config** (conservative, default, aggressive)
2. **Test di venue** sebelum pertandingan
3. **Catat kondisi lapangan** dan config yang cocok
4. **Siapkan backup** jika robot tidak stabil

---
*AROC26 Team - 2026*
