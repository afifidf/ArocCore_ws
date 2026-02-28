# Panduan Membuat Orbit Walking Sendiri (OP3)

## Step 1 — Pahami Cara Kerja Walking OP3

Robot OP3 berjalan dengan publish ke 2 topic ini:

```bash
# Kirim parameter gerakan
/robotis/walking/set_params  →  type: op3_walking_module_msgs/WalkingParam

# Mulai/stop jalan
/robotis/walking/command     →  type: std_msgs/String  ("start" / "stop")
```

Parameter penting di `WalkingParam`:

| Field | Fungsi |
|---|---|
| `x_move_amplitude` | maju (+) / mundur (-) dalam meter |
| `y_move_amplitude` | geser kiri (+) / kanan (-) dalam meter |
| `angle_move_amplitude` | rotasi dalam **radian** |
| `period_time` | durasi 1 langkah dalam detik |

---

## Step 2 — Pahami Logika Orbit

Orbit = **geser samping + rotasi bersamaan**

```
Orbit kanan:  y = negatif  +  angle = positif
Orbit kiri :  y = positif  +  angle = negatif
```

Sambil bola tetap di depan kepala (head tracking dari `headControl26`), robot mengorbit mengelilingi bola.

---

## Step 3 — Aktifkan Walking Module

Sebelum publish parameter, pastikan `walking_module` aktif:

```python
from std_msgs.msg import String

module_pub = node.create_publisher(String, '/robotis/enable_ctrl_module', 10)

msg = String()
msg.data = 'walking_module'
module_pub.publish(msg)

import time
time.sleep(0.5)  # tunggu module aktif
```

---

## Step 4 — Struktur Node Orbit Sederhana

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from op3_walking_module_msgs.msg import WalkingParam


class OrbitNode(Node):
    def __init__(self):
        super().__init__('orbit_node')

        # Publisher
        self.pub_module  = self.create_publisher(String,       '/robotis/enable_ctrl_module',  10)
        self.pub_param   = self.create_publisher(WalkingParam, '/robotis/walking/set_params',  10)
        self.pub_command = self.create_publisher(String,       '/robotis/walking/command',     10)

        # Subscriber: deteksi bola dari headControl26
        self.sub_ball = self.create_subscription(
            # sesuaikan dengan type topic /obj_detect
            # contoh pakai BoundingBox atau Point
            String,
            '/obj_detect',
            self.ball_callback,
            10
        )

        self.ball_detected = False
        self.timer = self.create_timer(0.1, self.loop)  # 10Hz

        # Aktifkan walking module
        self.enable_walking()

    def enable_walking(self):
        msg = String()
        msg.data = 'walking_module'
        self.pub_module.publish(msg)
        self.get_logger().info('Walking module enabled')

    def ball_callback(self, msg):
        # Update flag setiap ada data bola masuk
        self.ball_detected = True  # sesuaikan logic sesuai isi msg

    def loop(self):
        if self.ball_detected:
            self.do_orbit()
        else:
            self.stop_walking()

    def do_orbit(self, direction='right'):
        # Set parameter orbit
        param = WalkingParam()
        param.x_move_amplitude     = 0.010   # sedikit maju
        param.y_move_amplitude     = -0.030 if direction == 'right' else 0.030
        param.angle_move_amplitude = 0.122  if direction == 'right' else -0.122  # ~7 deg
        param.period_time          = 0.55   # detik per langkah

        self.pub_param.publish(param)
        self.walking_start()

    def walking_start(self):
        msg = String()
        msg.data = 'start'
        self.pub_command.publish(msg)

    def stop_walking(self):
        msg = String()
        msg.data = 'stop'
        self.pub_command.publish(msg)


def main():
    rclpy.init()
    node = OrbitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Step 5 — Nilai Parameter untuk Tuning

Ini nilai awal, sesuaikan dengan kondisi robot kamu:

| Parameter | Nilai Awal | Keterangan |
|---|---|---|
| `x_move_amplitude` | `0.010` | Sedikit maju agar radius tidak membesar |
| `y_move_amplitude` | `-0.030` | Geser kanan (negatif = kanan) |
| `angle_move_amplitude` | `0.122` | ±7 derajat dalam radian |
| `period_time` | `0.55` | Detik per langkah |

> **Tips tuning:**
> - Kalau robot terlalu cepat orbit → kecilkan `y_move_amplitude`
> - Kalau radius orbit membesar → tambah sedikit `x_move_amplitude`
> - Kalau robot oleng → besarkan `period_time` (langkah lebih lambat)

---

## Step 6 — Urutan Testing

```bash
# 1. Test publish manual dari terminal dulu
ros2 topic pub /robotis/walking/command std_msgs/String "data: 'start'"

# 2. Test set_params dari terminal
ros2 topic pub /robotis/walking/set_params op3_walking_module_msgs/WalkingParam \
"{x_move_amplitude: 0.01, y_move_amplitude: -0.03, angle_move_amplitude: 0.122, period_time: 0.55}"

# 3. Kalau robot bergerak → masukkan ke kode Python kamu

# 4. Tuning nilai sampai orbit terasa natural
```

---

## Step 7 — Integrasi dengan headControl26

Data bola dari `headControl26` dikirim lewat topic `/obj_detect`. Struktur datanya:

```
cx  → posisi horizontal bola di frame (pixel)
cy  → posisi vertikal bola di frame (pixel)
```

Logika sederhana:
```python
def ball_callback(self, msg):
    # Kalau cx ada → bola terdeteksi
    if msg.cx > 0 and msg.cy > 0:
        self.ball_detected = True
        self.last_seen = self.get_clock().now()
    else:
        # Bola hilang jika tidak terdeteksi selama 1 detik
        elapsed = (self.get_clock().now() - self.last_seen).nanoseconds / 1e9
        if elapsed > 1.0:
            self.ball_detected = False
```

---

## Alur Data Lengkap

```
[headControl26]
    ↓  /obj_detect  (cx, cy posisi bola)
[OrbitNode - buatan sendiri]
    ↓  ball_detected = True
    ↓  publish WalkingParam ke /robotis/walking/set_params
    ↓  publish "start" ke /robotis/walking/command
[op3_walking_module]
    ↓
[Robot berjalan orbit mengelilingi bola] ✅
```
