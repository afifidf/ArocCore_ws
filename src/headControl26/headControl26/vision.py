# loc: aroc26/src/headControl26/headControl26/vision.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import time
from ultralytics import YOLO
from rclpy.qos import qos_profile_sensor_data
from .configCam import CameraConfig #OPSIONAL, Kameraku Jelek

SET_CONF = 0.6  # threshold confidence deteksi bola (dipakai di model DAN filter manual)
FRAME_W = 640   # lebar frame (pixel) — harus sama dengan InRangeMax PAN di mainHeadControl
FRAME_H = 480   # tinggi frame (pixel) — harus sama dengan InRangeMax TILT di mainHeadControl


class UsbCamSubscriber(Node):
    def __init__(self):
        super().__init__('usb_cam_yolo')

        self.bridge = CvBridge()

        self.camera = CameraConfig("/dev/video0")
        self.camera.apply_default_settings()
        # =========================
        # LOAD MODEL OPENVINO
        # =========================
        self.model = YOLO(
            "/home/afifi/aroc26/src/headControl26/aroc_openvino_model",
            task="detect"
        )

        # FPS
        self.prev_time = 0.0
        self.fps = 0.0

        # Publisher hasil deteksi
        self.obj_pub = self.create_publisher(
            String,
            '/obj_detect',
            10
        )

        # Subscribe kamera
        self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info("✅ YOLO OpenVINO ROS2 Node Jalan")

    def image_callback(self, msg):
        try:
            # =========================
            # IMAGE CONVERT
            # =========================
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )

            # =========================
            # [FIX] Resize ke FRAME_W x FRAME_H (640x480) agar koordinat
            # cx, cy yang dipublish konsisten dengan asumsi resolusi di
            # mainHeadControl (PID range 0-640 pan, 0-480 tilt).
            # Sebelumnya: resize ke 450x337 → koordinat tidak sesuai range PID.
            # imgsz=640 sesuai ukuran frame yang diproses YOLO.
            # =========================
            frame = cv2.resize(frame, (FRAME_W, FRAME_H))
            results = self.model(
                frame,
                imgsz=960,          # [FIX v2] sesuai fixed shape model OpenVINO
                conf=SET_CONF,      # [FIX] threshold konsisten dengan SET_CONF
                device="cpu",
                verbose=False
            )

            r = results[0]
            annotated_frame = r.plot()

            detected_objects = []

            # =========================
            # AMBIL DETEKSI TERBAIK
            # =========================
            if r.boxes is not None and len(r.boxes) > 0:

                best_conf = 0.0
                best_cx = None
                best_cy = None

                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    class_name = r.names[cls_id]

                    if class_name != "bola":
                        continue

                    # [FIX] Filter hanya berdasarkan best_conf karena model sudah
                    # filter conf >= SET_CONF. Tidak perlu cek conf > SET_CONF lagi.
                    if conf > best_conf:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cx = (x1 + x2) // 2
                        cy = (y1 + y2) // 2

                        best_conf = conf
                        best_cx = cx
                        best_cy = cy

                # Setelah loop selesai, kirim hanya yang terbaik
                if best_cx is not None:
                    self.get_logger().info(
                        f"BEST DETECTION | conf={best_conf:.2f} | center=({best_cx},{best_cy})"
                    )
                    # Publish koordinat dalam frame FRAME_W x FRAME_H (640x480)
                    detected_objects.append(f"{best_cx},{best_cy}")

            # =========================
            # PUBLISH KE TOPIC
            # =========================
            if detected_objects:
                msg_out = String()
                msg_out.data = "\n".join(detected_objects)
                self.obj_pub.publish(msg_out)

            # =========================
            # HITUNG FPS
            # =========================
            curr_time = time.time()
            if self.prev_time != 0:
                self.fps = 1.0 / (curr_time - self.prev_time)
            self.prev_time = curr_time

            cv2.putText(
                annotated_frame,
                f"FPS: {self.fps:.2f}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2
            )

            # =========================
            # DISPLAY
            # =========================
            cv2.imshow("YOLO OpenVINO ROS2", annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")


def main():
    rclpy.init()
    node = UsbCamSubscriber()
    rclpy.spin(node)

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()