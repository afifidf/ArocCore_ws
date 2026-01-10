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
    
    Menampilkan 3 gambar secara side-by-side:
    ┌─────────────┬─────────────┬─────────────┐
    │   CAMERA    │    YOLO     │   FIELD     │
    │  (raw img)  │ (detection) │   (DLT)     │
    └─────────────┴─────────────┴─────────────┘
    
    Topics yang di-subscribe:
    - /usb_cam_node/image_raw       : Gambar mentah dari kamera
    - /ball_detector_node/image_out : Gambar dengan bounding box deteksi
    - /field_dlt/debug_image        : Gambar dengan garis lapangan (DLT)
    
    Tekan 'q' di window OpenCV untuk keluar.
    """

    def __init__(self) -> None:
        """
        Constructor - Inisialisasi node YoloViewer
        
        Setup yang dilakukan:
        1. CvBridge untuk konversi gambar
        2. Buffer untuk menyimpan frame terakhir dari setiap topic
        3. Subscriber untuk 3 topic gambar
        4. Timer untuk update tampilan pada 30 FPS
        """
        super().__init__("yolo_viewer")  # Nama node: 'yolo_viewer'

        # CvBridge untuk konversi antara ROS Image dan OpenCV Mat
        self.bridge = CvBridge()

        # Buffer untuk menyimpan frame terakhir dari setiap source
        # None berarti belum ada frame yang diterima
        self._last_raw = None       # Frame dari kamera raw
        self._last_det = None       # Frame dari deteksi YOLO
        self._last_field = None     # Frame dari field DLT

        # === SUBSCRIBER 1: Kamera Raw ===
        # Menerima gambar mentah langsung dari kamera USB
        self.raw_sub = self.create_subscription(
            Image,                          # Tipe message
            "/usb_cam_node/image_raw",      # Topic name
            self.raw_callback,              # Callback function
            10,                             # Queue size
        )

        # === SUBSCRIBER 2: Deteksi YOLO ===
        # Menerima gambar dengan bounding box hasil deteksi bola
        self.det_sub = self.create_subscription(
            Image,
            "/ball_detector_node/image_out",
            self.det_callback,
            10,
        )

        # === SUBSCRIBER 3: Field DLT ===
        # Menerima gambar dengan visualisasi garis lapangan dan corner
        self.field_sub = self.create_subscription(
            Image,
            "/field_dlt/debug_image",
            self.field_callback,
            10,
        )

        # === TIMER untuk Update Tampilan ===
        # Dipanggil setiap 1/30 detik (30 FPS) untuk update window OpenCV
        self.timer = self.create_timer(1.0 / 30.0, self.update_view)

        # Log info bahwa node sudah siap
        self.get_logger().info(
            "🖼️  YOLO + Field DLT Viewer started\n"
            "   Col 1 : /usb_cam_node/image_raw\n"
            "   Col 2 : /ball_detector_node/image_out\n"
            "   Col 3 : /field_dlt/debug_image\n"
            "   Press 'q' in the OpenCV window to quit."
        )

    # =========================================================================
    # CALLBACK FUNCTIONS
    # =========================================================================
    # Callback dipanggil otomatis oleh ROS2 ketika ada message baru di topic
    # =========================================================================

    def raw_callback(self, msg: Image) -> None:
        """
        Callback untuk menerima gambar raw dari kamera
        
        Args:
            msg: ROS Image message dari topic /usb_cam_node/image_raw
        """
        try:
            # Konversi ROS Image -> OpenCV Mat (format BGR 8-bit)
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._last_raw = img  # Simpan ke buffer
        except Exception as e:
            self.get_logger().error(f"Failed to convert raw image: {e}")

    def det_callback(self, msg: Image) -> None:
        """
        Callback untuk menerima gambar hasil deteksi YOLO
        
        Args:
            msg: ROS Image message dari topic /ball_detector_node/image_out
                 Gambar sudah memiliki bounding box dan label deteksi
        """
        try:
            # Konversi ROS Image -> OpenCV Mat
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._last_det = img  # Simpan ke buffer
        except Exception as e:
            self.get_logger().error(f"Failed to convert detection image: {e}")

    def field_callback(self, msg: Image) -> None:
        """
        Callback untuk menerima debug image dari node Field DLT
        
        Args:
            msg: ROS Image message dari topic /field_dlt/debug_image
                 Gambar dengan visualisasi garis lapangan dan corner points
        """
        try:
            # Konversi ROS Image -> OpenCV Mat
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._last_field = img  # Simpan ke buffer
        except Exception as e:
            self.get_logger().error(f"Failed to convert field DLT image: {e}")

    # =========================================================================
    # VISUALIZATION LOOP
    # =========================================================================
    # Timer callback untuk update tampilan OpenCV setiap 1/30 detik
    # =========================================================================

    def update_view(self) -> None:
        """
        Update tampilan window OpenCV dengan gambar terbaru
        
        Proses:
        1. Cek apakah ada gambar yang tersedia
        2. Copy buffer untuk menghindari race condition
        3. Resize semua gambar ke tinggi yang sama
        4. Gabungkan secara horizontal (side-by-side)
        5. Tampilkan di window OpenCV
        6. Cek input keyboard untuk quit
        """
        # Skip jika belum ada gambar sama sekali
        if self._last_raw is None and self._last_det is None and self._last_field is None:
            return

        # Copy buffer supaya aman dari race condition dengan callback
        # (callback bisa mengubah buffer saat kita sedang memproses)
        raw = self._last_raw.copy() if self._last_raw is not None else None
        det = self._last_det.copy() if self._last_det is not None else None
        fld = self._last_field.copy() if self._last_field is not None else None

        # Kumpulkan gambar yang tersedia beserta labelnya
        imgs = []       # List gambar
        labels = []     # List label untuk setiap gambar

        if raw is not None:
            imgs.append(raw)
            labels.append("CAM")        # Label: Camera raw
        if det is not None:
            imgs.append(det)
            labels.append("YOLO")       # Label: YOLO detection
        if fld is not None:
            imgs.append(fld)
            labels.append("FIELD")      # Label: Field DLT

        # Skip jika tidak ada gambar
        if not imgs:
            return

        # Resize semua gambar ke tinggi yang sama untuk display yang rapi
        target_h = 360  # Tinggi target (pixel) - 360 untuk performa ringan
        resized_imgs = []
        
        for i, im in enumerate(imgs):
            # Resize gambar dengan mempertahankan aspect ratio
            r = self._resize_to_height(im, target_h)

            # Tambahkan label text di pojok kiri atas gambar
            cv2.putText(
                r,                          # Gambar target
                labels[i],                  # Text label
                (10, 20),                   # Posisi (x, y)
                cv2.FONT_HERSHEY_SIMPLEX,   # Font
                0.6,                        # Ukuran font
                (0, 255, 255),              # Warna (BGR: Cyan)
                2,                          # Ketebalan garis
            )
            resized_imgs.append(r)

        # Samakan lebar minimum supaya bisa di-stack dengan rapi
        min_w = min(r.shape[1] for r in resized_imgs)
        resized_imgs = [r[:, :min_w] for r in resized_imgs]  # Crop ke lebar minimum

        # Gabungkan semua gambar secara horizontal (side-by-side)
        canvas = np.hstack(resized_imgs)

        # Tampilkan di window OpenCV
        cv2.imshow("OP3 Camera + YOLO + Field DLT", canvas)
        
        # Cek input keyboard (waitKey return ASCII code)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):  # Tekan 'q' untuk quit
            self.get_logger().info("👋 Viewer closed by user")
            cv2.destroyAllWindows()     # Tutup semua window OpenCV
            rclpy.shutdown()            # Shutdown ROS2 dari dalam node

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
