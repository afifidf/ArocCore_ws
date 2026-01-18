# OP3 Gazebo Simulation (ROS2)

Package simulasi Gazebo untuk robot ROBOTIS OP3 pada ROS2.

## Persyaratan

```bash
# Install Gazebo (Ignition/Gazebo Sim)
sudo apt install ros-humble-ros-gz  # atau ros-jazzy-ros-gz

# Install dependencies
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-xacro
```

## Cara Menjalankan

### 1. Build Package

```bash
cd ~/aroc26_core
colcon build --packages-select op3_gazebo op3_description
source install/setup.bash
```

### 2. Jalankan Simulasi (Simple Version)

```bash
# Versi sederhana - untuk testing awal
ros2 launch op3_gazebo op3_gazebo_simple.launch.py
```

Ini akan:
- Membuka Gazebo dengan world kosong + bola
- Spawn robot OP3
- Membuka Joint State Publisher GUI (untuk kontrol manual)

### 3. Jalankan Simulasi (Full Version)

```bash
# Versi lengkap dengan ros2_control
ros2 launch op3_gazebo op3_gazebo.launch.py
```

## Struktur Package

```
op3_gazebo/
├── launch/
│   ├── op3_gazebo.launch.py        # Launch lengkap dengan ros2_control
│   └── op3_gazebo_simple.launch.py # Launch simple untuk testing
├── config/
│   ├── op3_controllers.yaml        # Konfigurasi ros2_control
│   └── initial_pose.yaml           # Posisi bringup robot
├── worlds/
│   └── op3_empty.world             # World dengan lantai + bola
├── scripts/
│   └── initial_pose.py             # Script untuk initial pose
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Initial Pose (Bringup)

Robot akan dikirim ke posisi bringup (berdiri siap) dengan konfigurasi:

| Joint | Posisi (deg) | Deskripsi |
|-------|--------------|-----------|
| r/l_sho_pitch | ±15 | Bahu pitch |
| r/l_sho_roll | ∓45 | Bahu roll |
| r/l_el | ±45 | Siku |
| r/l_hip_pitch | ±70 | Hip pitch |
| r/l_knee | ∓142 | Lutut |
| r/l_ank_pitch | ∓70 | Ankle pitch |
| head_tilt | -10 | Kepala menunduk sedikit |

## Troubleshooting

### Gazebo tidak mau start
```bash
# Cek apakah Gazebo terinstall
gz sim --version

# Jika belum, install:
sudo apt install ros-humble-ros-gz-sim
```

### Robot tidak muncul di Gazebo
```bash
# Cek apakah URDF valid
ros2 run xacro xacro ~/aroc26_core/src/ROBOTIS-OP3-Common/op3_description/urdf/robotis_op3.urdf.xacro

# Cek topic robot_description
ros2 topic echo /robot_description
```

### Error "package not found"
```bash
# Pastikan sudah source
source ~/aroc26_core/install/setup.bash
```

## Author

AROC26 Team - 2026
