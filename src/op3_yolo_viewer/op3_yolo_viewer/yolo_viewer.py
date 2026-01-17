# =============================================================================
# op3_yolo_viewer/yolo_viewer.py
# =============================================================================
# Node ROS2 untuk menampilkan hasil deteksi YOLO secara real-time
# Menggabungkan 3 view: Kamera Raw, Deteksi YOLO, dan Field DLT
# 
# Author: Gabriel, Afifi, Rehan (+ field DLT integration)
# =============================================================================

# =============================================================================
# IMPORT LIBRARIES
# =============================================================================
import cv2                          # OpenCV untuk pemrosesan dan tampilan gambar
import numpy as np                  # NumPy untuk operasi array
import time                         # Untuk FPS calculation
import os                           # Untuk membuat folder screenshot
from datetime import datetime       # Untuk timestamp screenshot

import rclpy                        # Library utama ROS2 Python
from rclpy.node import Node         # Base class untuk node ROS2

from sensor_msgs.msg import Image   # Message type untuk gambar
from cv_bridge import CvBridge      # Konversi ROS Image <-> OpenCV


# =============================================================================
# CLASS UTAMA: YoloViewer
# =============================================================================
class YoloViewer(Node):
    """
    Node untuk menampilkan hasil deteksi secara real-time dalam satu window.
    
    Menampilkan 2 gambar secara side-by-side:
    ┌─────────────┬─────────────┐
    │   CAMERA    │    YOLO     │
    │  (raw img)  │ (detection) │
    └─────────────┴─────────────┘
    
    Topics yang di-subscribe (configurable via parameters):
    - raw_image_topic   : Gambar mentah dari kamera
    - yolo_image_topic  : Gambar dengan bounding box deteksi
    
    Keyboard Controls:
    - 'q' : Quit / keluar dari viewer
    - 'p' / SPACE : Pause / resume tampilan
    - 's' : Screenshot (simpan gambar saat ini)
    - '1' : Toggle view kamera raw ON/OFF
    - '2' : Toggle view YOLO detection ON/OFF
    - 'r' : Reset (tampilkan semua view)
    - 'f' : Toggle FPS display ON/OFF
    """

    def __init__(self) -> None:
        """
        Constructor - Inisialisasi node YoloViewer
        
        Setup yang dilakukan:
        1. Parameter ROS2 untuk konfigurasi topics
        2. CvBridge untuk konversi gambar
        3. Buffer untuk menyimpan frame terakhir dari setiap topic
        4. Subscriber untuk 3 topic gambar
        5. Timer untuk update tampilan
        6. FPS tracking untuk setiap stream
        7. Keyboard control state
        """
        super().__init__("yolo_viewer")  # Nama node: 'yolo_viewer'

        # === PARAMETER ROS2 ===
        # Deklarasi parameter untuk topic names (dapat di-override via launch file)
        self._init_parameters()

        # CvBridge untuk konversi antara ROS Image dan OpenCV Mat
        self.bridge = CvBridge()

        # Buffer untuk menyimpan frame terakhir dari setiap source
        # None berarti belum ada frame yang diterima
        self._last_raw = None       # Frame dari kamera raw
        self._last_det = None       # Frame dari deteksi YOLO

        # === FPS TRACKING ===
        self._init_fps_tracking()

        # === KEYBOARD CONTROL STATE ===
        self._init_control_state()

        # === SUBSCRIBERS ===
        self._init_subscribers()

        # === TIMER untuk Update Tampilan ===
        # Dipanggil setiap 1/target_fps detik untuk update window OpenCV
        self.timer = self.create_timer(1.0 / self.target_fps, self.update_view)

        # Log info bahwa node sudah siap
        self._log_startup_info()

    def _init_parameters(self) -> None:
        """Inisialisasi parameter ROS2 untuk konfigurasi topics dan display"""
        # Deklarasi parameter dengan nilai default
        # Note: Default topic sesuai dengan usb_cam launch (namespace: camera1)
        params = [
            ('raw_image_topic', '/camera1/image_raw'),
            ('yolo_image_topic', '/ball_detector_node/image_out'),
            ('target_fps', 30),
            ('display_height', 360),
            ('screenshot_dir', './screenshots'),
            ('show_fps', True),
        ]
        
        for name, default in params:
            self.declare_parameter(name, default)
        
        # Load parameter values
        self.raw_topic = self.get_parameter('raw_image_topic').value
        self.yolo_topic = self.get_parameter('yolo_image_topic').value
        self.target_fps = self.get_parameter('target_fps').value
        self.display_height = self.get_parameter('display_height').value
        self.screenshot_dir = self.get_parameter('screenshot_dir').value
        self.show_fps = self.get_parameter('show_fps').value

    def _init_fps_tracking(self) -> None:
        """Inisialisasi FPS tracking untuk setiap stream"""
        # FPS counters untuk setiap stream
        self._fps_data = {
            'raw': {'count': 0, 'fps': 0.0, 'last_time': time.time()},
            'yolo': {'count': 0, 'fps': 0.0, 'last_time': time.time()},
            'display': {'count': 0, 'fps': 0.0, 'last_time': time.time()},
        }
        self._fps_update_interval = 1.0  # Update FPS setiap 1 detik

    def _init_control_state(self) -> None:
        """Inisialisasi state untuk keyboard controls"""
        self.is_paused = False          # Pause/resume state
        self.show_raw = True            # Toggle view kamera raw
        self.show_yolo = True           # Toggle view YOLO
        self._last_canvas = None        # Simpan canvas terakhir untuk pause mode

    def _init_subscribers(self) -> None:
        """Inisialisasi ROS2 subscribers untuk image topics"""
        # === SUBSCRIBER 1: Kamera Raw ===
        self.raw_sub = self.create_subscription(
            Image,
            self.raw_topic,
            self.raw_callback,
            10,
        )

        # === SUBSCRIBER 2: Deteksi YOLO ===
        self.det_sub = self.create_subscription(
            Image,
            self.yolo_topic,
            self.det_callback,
            10,
        )

    def _log_startup_info(self) -> None:
        """Log informasi startup dan keyboard controls"""
        self.get_logger().info(
            "🖼️  YOLO Viewer started\n"
            f"   Col 1 : {self.raw_topic}\n"
            f"   Col 2 : {self.yolo_topic}\n"
            "   ─────────────────────────────────\n"
            "   Keyboard Controls:\n"
            "   [q]     : Quit\n"
            "   [p/SPC] : Pause/Resume\n"
            "   [s]     : Screenshot\n"
            "   [1/2]   : Toggle view 1/2\n"
            "   [r]     : Reset all views\n"
            "   [f]     : Toggle FPS display"
        )

    # =========================================================================
    # CALLBACK FUNCTIONS
    # =========================================================================
    # Callback dipanggil otomatis oleh ROS2 ketika ada message baru di topic
    # =========================================================================

    def _update_fps(self, stream_name: str) -> None:
        """
        Update FPS counter untuk stream tertentu
        
        Args:
            stream_name: Nama stream ('raw', 'yolo', 'field', 'display')
        """
        data = self._fps_data[stream_name]
        data['count'] += 1
        
        current_time = time.time()
        elapsed = current_time - data['last_time']
        
        if elapsed >= self._fps_update_interval:
            data['fps'] = data['count'] / elapsed
            data['count'] = 0
            data['last_time'] = current_time

    def raw_callback(self, msg: Image) -> None:
        """
        Callback untuk menerima gambar raw dari kamera
        
        Args:
            msg: ROS Image message dari raw_image_topic
        """
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._last_raw = img
            self._update_fps('raw')
        except Exception as e:
            self.get_logger().error(f"Failed to convert raw image: {e}")

    def det_callback(self, msg: Image) -> None:
        """
        Callback untuk menerima gambar hasil deteksi YOLO
        
        Args:
            msg: ROS Image message dari yolo_image_topic
        """
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._last_det = img
            self._update_fps('yolo')
        except Exception as e:
            self.get_logger().error(f"Failed to convert detection image: {e}")

    # =========================================================================
    # VISUALIZATION LOOP
    # =========================================================================
    # Timer callback untuk update tampilan OpenCV
    # =========================================================================

    def update_view(self) -> None:
        """
        Update tampilan window OpenCV dengan gambar terbaru
        
        Proses:
        1. Handle keyboard input
        2. Cek pause state
        3. Cek apakah ada gambar yang tersedia
        4. Copy buffer untuk menghindari race condition
        5. Resize semua gambar ke tinggi yang sama
        6. Tambahkan FPS overlay
        7. Gabungkan secara horizontal (side-by-side)
        8. Tampilkan di window OpenCV
        """
        # Handle keyboard input terlebih dahulu
        if not self._handle_keyboard():
            return  # Node di-shutdown

        # Jika paused, tampilkan canvas terakhir
        if self.is_paused:
            if self._last_canvas is not None:
                self._display_canvas(self._last_canvas)
            return

        # Skip jika belum ada gambar sama sekali
        if self._last_raw is None and self._last_det is None:
            return

        # Copy buffer supaya aman dari race condition dengan callback
        raw = self._last_raw.copy() if self._last_raw is not None else None
        det = self._last_det.copy() if self._last_det is not None else None

        # Kumpulkan gambar yang tersedia beserta labelnya (berdasarkan toggle state)
        imgs = []
        labels = []
        fps_values = []

        if raw is not None and self.show_raw:
            imgs.append(raw)
            labels.append("CAM")
            fps_values.append(self._fps_data['raw']['fps'])
        if det is not None and self.show_yolo:
            imgs.append(det)
            labels.append("YOLO")
            fps_values.append(self._fps_data['yolo']['fps'])

        # Skip jika tidak ada gambar untuk ditampilkan
        if not imgs:
            # Tampilkan placeholder jika semua view di-toggle off
            self._display_placeholder()
            return

        # Resize semua gambar ke tinggi yang sama
        resized_imgs = []
        
        for i, im in enumerate(imgs):
            r = self._resize_to_height(im, self.display_height)
            
            # Tambahkan label dan FPS overlay
            self._draw_overlay(r, labels[i], fps_values[i])
            resized_imgs.append(r)

        # Samakan lebar minimum supaya bisa di-stack dengan rapi
        min_w = min(r.shape[1] for r in resized_imgs)
        resized_imgs = [r[:, :min_w] for r in resized_imgs]

        # Gabungkan semua gambar secara horizontal
        canvas = np.hstack(resized_imgs)

        # Tambahkan status bar di bagian bawah
        canvas = self._add_status_bar(canvas)

        # Update display FPS
        self._update_fps('display')

        # Simpan canvas untuk pause mode
        self._last_canvas = canvas.copy()

        # Tampilkan
        self._display_canvas(canvas)

    def _handle_keyboard(self) -> bool:
        """
        Handle keyboard input
        
        Returns:
            False jika node harus di-shutdown, True jika lanjut
        """
        key = cv2.waitKey(1) & 0xFF
        
        if key == 255:  # No key pressed
            return True
        
        if key == ord('q'):  # Quit
            self.get_logger().info("👋 Viewer closed by user")
            cv2.destroyAllWindows()
            rclpy.shutdown()
            return False
        
        elif key == ord('p') or key == ord(' '):  # Pause/Resume (p or SPACE)
            self.is_paused = not self.is_paused
            status = "⏸️  PAUSED" if self.is_paused else "▶️  RESUMED"
            self.get_logger().info(status)
        
        elif key == ord('s'):  # Screenshot
            self._take_screenshot()
        
        elif key == ord('1'):  # Toggle raw view
            self.show_raw = not self.show_raw
            status = "ON" if self.show_raw else "OFF"
            self.get_logger().info(f"🎥 Camera view: {status}")
        
        elif key == ord('2'):  # Toggle YOLO view
            self.show_yolo = not self.show_yolo
            status = "ON" if self.show_yolo else "OFF"
            self.get_logger().info(f"🎯 YOLO view: {status}")
        
        elif key == ord('r'):  # Reset all views
            self.show_raw = True
            self.show_yolo = True
            self.get_logger().info("🔄 All views reset to ON")
        
        elif key == ord('f'):  # Toggle FPS display
            self.show_fps = not self.show_fps
            status = "ON" if self.show_fps else "OFF"
            self.get_logger().info(f"📊 FPS display: {status}")
        
        return True

    def _draw_overlay(self, img: np.ndarray, label: str, fps: float) -> None:
        """
        Draw label and FPS overlay pada gambar
        
        Args:
            img: Gambar target (akan dimodifikasi in-place)
            label: Label text (CAM, YOLO, FIELD)
            fps: FPS value untuk stream ini
        """
        # Background semi-transparent untuk text
        overlay_height = 50 if self.show_fps else 30
        cv2.rectangle(img, (0, 0), (img.shape[1], overlay_height), (0, 0, 0), -1)
        cv2.rectangle(img, (0, 0), (img.shape[1], overlay_height), (0, 0, 0), -1)
        
        # Blend dengan alpha
        alpha = 0.6
        img[0:overlay_height, :] = cv2.addWeighted(
            img[0:overlay_height, :], alpha,
            np.zeros((overlay_height, img.shape[1], 3), dtype=np.uint8), 1 - alpha, 0
        )
        
        # Label text
        cv2.putText(
            img, label, (10, 22),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
        )
        
        # FPS text (jika enabled)
        if self.show_fps:
            fps_text = f"FPS: {fps:.1f}"
            cv2.putText(
                img, fps_text, (10, 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1
            )

    def _add_status_bar(self, canvas: np.ndarray) -> np.ndarray:
        """
        Tambahkan status bar di bagian bawah canvas
        
        Args:
            canvas: Canvas gabungan semua gambar
            
        Returns:
            Canvas dengan status bar
        """
        bar_height = 25
        bar = np.zeros((bar_height, canvas.shape[1], 3), dtype=np.uint8)
        
        # Status text
        display_fps = self._fps_data['display']['fps']
        views_status = f"Views: [1]CAM={'ON' if self.show_raw else 'OFF'} [2]YOLO={'ON' if self.show_yolo else 'OFF'}"
        controls = "| [P]ause [S]creenshot [R]eset [F]PS [Q]uit"
        
        status_text = f"Display: {display_fps:.1f} FPS | {views_status} {controls}"
        
        cv2.putText(
            bar, status_text, (10, 18),
            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1
        )
        
        # Pause indicator
        if self.is_paused:
            cv2.putText(
                bar, "|| PAUSED", (canvas.shape[1] - 100, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2
            )
        
        return np.vstack([canvas, bar])

    def _display_placeholder(self) -> None:
        """Tampilkan placeholder ketika semua view di-toggle off"""
        placeholder = np.zeros((self.display_height, 480, 3), dtype=np.uint8)
        
        text = "All views disabled"
        text2 = "Press 1/2/3 to enable or R to reset"
        
        cv2.putText(
            placeholder, text, (80, self.display_height // 2 - 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (100, 100, 100), 2
        )
        cv2.putText(
            placeholder, text2, (40, self.display_height // 2 + 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1
        )
        
        canvas = self._add_status_bar(placeholder)
        self._display_canvas(canvas)

    def _display_canvas(self, canvas: np.ndarray) -> None:
        """Display canvas pada window OpenCV"""
        cv2.imshow("OP3 Camera + YOLO Viewer", canvas)

    def _take_screenshot(self) -> None:
        """Simpan screenshot dari tampilan saat ini"""
        if self._last_canvas is None:
            self.get_logger().warn("⚠️  No image to screenshot")
            return
        
        # Buat folder jika belum ada
        os.makedirs(self.screenshot_dir, exist_ok=True)
        
        # Generate filename dengan timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"screenshot_{timestamp}.png"
        filepath = os.path.join(self.screenshot_dir, filename)
        
        # Simpan gambar
        cv2.imwrite(filepath, self._last_canvas)
        self.get_logger().info(f"📸 Screenshot saved: {filepath}")

    @staticmethod
    def _resize_to_height(img: np.ndarray, target_h: int) -> np.ndarray:
        """
        Resize gambar ke tinggi tertentu dengan mempertahankan aspect ratio
        
        Args:
            img: Gambar input (OpenCV Mat / NumPy array)
            target_h: Tinggi target dalam pixel
            
        Returns:
            Gambar yang sudah di-resize
        """
        h, w = img.shape[:2]            # Ambil tinggi dan lebar asli
        
        if h == target_h:               # Jika sudah sama, tidak perlu resize
            return img
            
        scale = target_h / float(h)     # Hitung scale factor
        new_w = int(w * scale)          # Hitung lebar baru (proporsional)
        
        # Resize dengan interpolasi INTER_AREA (bagus untuk downscaling)
        return cv2.resize(img, (new_w, target_h), interpolation=cv2.INTER_AREA)


# =============================================================================
# MAIN FUNCTION
# =============================================================================
def main(args=None) -> None:
    """
    Entry point untuk menjalankan node YoloViewer
    
    Cara menjalankan:
    - ros2 run op3_yolo_viewer yolo_viewer
    
    Args:
        args: Command line arguments (optional)
    """
    # Inisialisasi ROS2
    rclpy.init(args=args)
    
    # Buat instance node
    node = YoloViewer()
    
    try:
        # Spin node (blocking - menunggu callback)
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle Ctrl+C
        pass
    finally:
        # Cleanup
        node.destroy_node()             # Destroy node ROS2
        cv2.destroyAllWindows()         # Tutup semua window OpenCV
        if rclpy.ok():
            rclpy.shutdown()            # Shutdown ROS2


# Entry point jika dijalankan langsung (python3 yolo_viewer.py)
if __name__ == "__main__":
    main()
