# OP3 Kick Module

Modul tendangan presisi untuk robot humanoid ROBOTIS OP3.

## Fitur

- **Trajectory Planning** - Menggunakan minimum jerk trajectory untuk gerakan tendangan yang smooth
- **Multiple Kick Types** - Support berbagai tipe tendangan:
  - `SHORT_PASS` - Umpan pendek dengan kekuatan rendah
  - `LONG_PASS` - Umpan jauh dengan kekuatan sedang
  - `SHOOT` - Tendangan keras ke gawang
  - `SIDE_KICK` - Tendangan samping
- **Balance Compensation** - Kompensasi keseimbangan real-time menggunakan IMU
- **Pre-kick Alignment** - Sistem alignment otomatis sebelum tendang
- **Configurable Parameters** - Semua parameter dapat di-tune via YAML

## Struktur Package

```
op3_kick_module/
├── include/op3_kick_module/
│   ├── kick_types.h        # Definisi tipe data dan enum
│   ├── kick_trajectory.h   # Trajectory generator
│   ├── kick_planner.h      # Kick planning
│   └── kick_controller.h   # Main controller
├── src/
│   ├── kick_trajectory.cpp
│   ├── kick_planner.cpp
│   ├── kick_controller.cpp
│   └── kick_module_node.cpp
├── config/
│   └── kick_config.yaml    # Konfigurasi parameter
├── launch/
│   ├── kick_module.launch.py
│   └── kick_demo.launch.py
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Instalasi

```bash
cd ~/aroc26_core
colcon build --packages-select op3_kick_module
source install/setup.bash
```

## Penggunaan

### Menjalankan Node

```bash
# Dengan config default
ros2 launch op3_kick_module kick_module.launch.py

# Dengan config custom
ros2 launch op3_kick_module kick_module.launch.py config_file:=/path/to/config.yaml
```

### Mengirim Command Tendangan

```bash
# Short pass
ros2 topic pub --once /robotis/kick/command std_msgs/msg/String "data: 'kick_short'"

# Long pass
ros2 topic pub --once /robotis/kick/command std_msgs/msg/String "data: 'kick_long'"

# Shoot
ros2 topic pub --once /robotis/kick/command std_msgs/msg/String "data: 'kick_shoot'"

# Auto kick (tipe ditentukan planner)
ros2 topic pub --once /robotis/kick/command std_msgs/msg/String "data: 'kick_auto'"

# Cancel
ros2 topic pub --once /robotis/kick/command std_msgs/msg/String "data: 'cancel'"
```

### Monitoring Status

```bash
# Lihat status tendangan
ros2 topic echo /robotis/kick/status

# Lihat notifikasi selesai
ros2 topic echo /robotis/kick/done
```

## ROS2 Topics

### Subscribed Topics

| Topic | Type | Deskripsi |
|-------|------|-----------|
| `/robotis/open_cr/imu` | `sensor_msgs/Imu` | Data IMU untuk balance |
| `/robotis/present_joint_states` | `sensor_msgs/JointState` | State joint saat ini |
| `/robotis/ball_position` | `geometry_msgs/Point` | Posisi bola (x, y, confidence) |
| `/robotis/kick/command` | `std_msgs/String` | Command tendangan |

### Published Topics

| Topic | Type | Deskripsi |
|-------|------|-----------|
| `/robotis/kick/joint_command` | `sensor_msgs/JointState` | Command ke joint |
| `/robotis/kick/status` | `std_msgs/String` | Status fase tendangan |
| `/robotis/kick/done` | `std_msgs/Bool` | Notifikasi tendangan selesai |

## Fase Tendangan

1. **IDLE** - Menunggu command
2. **ALIGNING** - Menyesuaikan posisi terhadap bola
3. **STABILIZING** - Menunggu robot stabil (IMU)
4. **LIFT_LEG** - Mengangkat kaki tendang
5. **SWING_BACK** - Ayunan ke belakang
6. **SWING_FORWARD** - Ayunan ke depan (kontak bola)
7. **FOLLOW_THROUGH** - Lanjutan setelah kontak
8. **RECOVERY** - Kembali ke posisi normal
9. **COMPLETED** - Tendangan selesai
10. **FAILED** - Tendangan gagal

## Konfigurasi

Edit `config/kick_config.yaml` untuk menyesuaikan parameter:

```yaml
kick_types:
  short_pass:
    strength: 0.4        # Kekuatan (0.0 - 1.0)
    swing_back: 15.0     # Sudut ayunan (derajat)
    swing_time: 0.3      # Waktu ayunan (detik)

alignment:
  distance_to_ball: 0.18  # Jarak ideal ke bola (meter)
  angle_tolerance: 8.0    # Toleransi sudut (derajat)

balance:
  ankle_roll_gain: 0.7    # Gain kompensasi roll
  ankle_pitch_gain: 0.9   # Gain kompensasi pitch
```

## Integrasi dengan Soccer Demo

Untuk mengintegrasikan dengan `op3_demo`, modifikasi `soccer_demo.cpp`:

```cpp
// Di handleKick(), ganti playMotion() dengan:
#include "op3_kick_module/kick_controller.h"

void SoccerDemo::handleKick() {
  // Kirim command ke kick module
  auto kick_cmd_pub = node_->create_publisher<std_msgs::msg::String>(
    "/robotis/kick/command", 10);
  
  std_msgs::msg::String msg;
  msg.data = "kick_auto";
  kick_cmd_pub->publish(msg);
  
  // Tunggu tendangan selesai via /robotis/kick/done
}
```

## Troubleshooting

### Robot tidak stabil saat menendang
- Turunkan nilai `balance.*_gain` di config
- Tingkatkan `stability_threshold` di alignment

### Tendangan tidak kena bola
- Sesuaikan `distance_to_ball` di alignment
- Kurangi `angle_tolerance` untuk presisi lebih tinggi

### Gerakan tendangan terlalu lambat/cepat
- Sesuaikan `swing_time` di kick_types
- Sesuaikan `contact_velocity` untuk kecepatan kontak

## Author

AROC26 Team - 2026

## License

Apache License 2.0
