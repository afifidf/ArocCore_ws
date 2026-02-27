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

SET_CONF       = 0.6   # threshold confidence deteksi bola
SET_CONF_GOAL  = 0.5   # threshold confidence deteksi gawang (lebih rendah karena gawang lebih susah)
FRAME_W        = 640   # lebar frame (pixel) — harus sama dengan InRangeMax PAN di mainHeadControl
FRAME_H        = 480   # tinggi frame (pixel) — harus sama dengan InRangeMax TILT di mainHeadControl

# Nama class di model YOLO
CLASS_BALL = "bola"
CLASS_GOAL = "Gawang"   # pastikan nama class di model YOLO kamu sama persis


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

        # Publisher hasil deteksi BOLA
        self.obj_pub = self.create_publisher(
            String,
            '/obj_detect',
            10
        )

        # Publisher hasil deteksi GAWANG
        # Format: "cx,cy,w,h" → dipakai oleh goal_alignment.py
        self.goal_pub = self.create_publisher(
            String,
            '/obj_detect_goal',
            10
        )

        # Publisher bbox bola lengkap (cx,cy,w,h) → dipakai task_control untuk estimasi jarak
        self.ball_bbox_pub = self.create_publisher(
            String,
            '/obj_detect_ball_bbox',
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
            # Pisahkan antara bola dan gawang dalam satu loop
            # =========================

            # ── BOLA ──────────────────────────────────────
            best_ball_conf = 0.0
            best_ball_cx   = None
            best_ball_cy   = None
            best_ball_w    = None
            best_ball_h    = None

            # ── GAWANG ────────────────────────────────────
            best_goal_conf = 0.0
            best_goal_cx   = None
            best_goal_cy   = None
            best_goal_w    = None
            best_goal_h    = None

            if r.boxes is not None and len(r.boxes) > 0:
                for box in r.boxes:
                    cls_id     = int(box.cls[0])
                    conf       = float(box.conf[0])
                    class_name = r.names[cls_id]
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2
                    w  = x2 - x1
                    h  = y2 - y1

                    # ── Deteksi BOLA ──────────────────────
                    if class_name == CLASS_BALL and conf >= SET_CONF:
                        if conf > best_ball_conf:
                            best_ball_conf = conf
                            best_ball_cx   = cx
                            best_ball_cy   = cy
                            best_ball_w    = w
                            best_ball_h    = h

                    # ── Deteksi GAWANG ────────────────────
                    elif class_name == CLASS_GOAL and conf >= SET_CONF_GOAL:
                        if conf > best_goal_conf:
                            best_goal_conf = conf
                            best_goal_cx   = cx
                            best_goal_cy   = cy
                            best_goal_w    = w
                            best_goal_h    = h

            # =========================
            # PUBLISH BOLA → /obj_detect
            # Format: "cx,cy"
            # =========================
            if best_ball_cx is not None:
                self.get_logger().info(
                    f"[BOLA] conf={best_ball_conf:.2f} | center=({best_ball_cx},{best_ball_cy})"
                )
                detected_objects.append(f"{best_ball_cx},{best_ball_cy}")
                # Publish bbox lengkap untuk estimasi jarak
                ball_bbox_msg = String()
                ball_bbox_msg.data = f"{best_ball_cx},{best_ball_cy},{best_ball_w},{best_ball_h}"
                self.ball_bbox_pub.publish(ball_bbox_msg)

            if detected_objects:
                msg_out = String()
                msg_out.data = "\n".join(detected_objects)
                self.obj_pub.publish(msg_out)

            # =========================
            # PUBLISH GAWANG → /obj_detect_goal
            # Format: "cx,cy,w,h"
            # Jika tidak terdeteksi → publish string kosong
            # agar goal_alignment.py tau gawang tidak kelihatan
            # =========================
            goal_msg = String()
            if best_goal_cx is not None:
                goal_msg.data = f"{best_goal_cx},{best_goal_cy},{best_goal_w},{best_goal_h}"
                self.get_logger().info(
                    f"[GAWANG] conf={best_goal_conf:.2f} | "
                    f"center=({best_goal_cx},{best_goal_cy}) | "
                    f"size=({best_goal_w}x{best_goal_h})"
                )
            else:
                goal_msg.data = ""   # kosong = gawang tidak terdeteksi
            self.goal_pub.publish(goal_msg)

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