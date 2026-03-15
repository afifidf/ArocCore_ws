# vision_cv.py — Deteksi Bola & Gawang via OpenCV (pengganti YOLO)
# Publish: /obj_detect, /obj_detect_ball_bbox, /obj_detect_goal

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time
from rclpy.qos import qos_profile_sensor_data
from .vision_utils import ColorBased, TO_HSV, TO_YUV, MORPH_CLOSE, ELE_ELLIPSE, RET_TREE, DRAW_CIRCLE
from .vision_image import VisionImage

FRAME_W = 640
FRAME_H = 480
BALL_MIN_AREA = 100
GOAL_MIN_AREA = 500

CALIB_DIR = os.path.dirname(os.path.abspath(__file__))


class VisionCvNode(Node):
    def __init__(self):
        super().__init__('vision_cv_node')

        self.bridge = CvBridge()
        self.declare_parameter('show_gui', True)
        self._show_gui = bool(self.get_parameter('show_gui').value)

        # Instance ColorBased untuk bola dan lapangan
        self.ball_detector = ColorBased()
        self.ball_detector.blobSetParams()
        self.field_detector = ColorBased()
        self.goal_detector = ColorBased()

        # Load kalibrasi HSV dari file
        try:
            self.val_ball = self.ball_detector.load(
                os.path.join(CALIB_DIR, "robotis_orange_ball"))
            self.get_logger().info(f"Ball HSV: {self.val_ball}")
        except (OSError, SyntaxError, ValueError) as e:
            self.get_logger().warn(f"Gagal load ball HSV: {e}, pakai default")
            self.val_ball = [[3, 68, 21], [33, 255, 243]]

        try:
            self.val_field = self.field_detector.load(
                os.path.join(CALIB_DIR, "robotis_green_field"))
            self.get_logger().info(f"Field HSV: {self.val_field}")
        except (OSError, SyntaxError, ValueError) as e:
            self.get_logger().warn(f"Gagal load field HSV: {e}, pakai default")
            self.val_field = [[65, 75, 59], [230, 120, 208]]

        # Publisher
        self.obj_pub = self.create_publisher(String, '/obj_detect', 10)
        self.goal_pub = self.create_publisher(String, '/obj_detect_goal', 10)
        self.ball_bbox_pub = self.create_publisher(String, '/obj_detect_ball_bbox', 10)

        self.create_subscription(
            Image, '/image_raw',
            self.image_callback, qos_profile_sensor_data)

        self.prev_time = 0.0
        self.fps = 0.0
        self._ball_lost_count = 0
        self._BALL_LOST_THRESHOLD = 30

        self.get_logger().info("✅ Vision OpenCV Node Started")

    def _detect_ball(self, frame):
        """Green field masking → HSV ball masking → contour/blob detection."""
        blurred = VisionImage.blur(frame, sigma=5)

        # Green field masking (YUV) — isolasi area lapangan saja
        field_mask = self.field_detector.mask(
            self.field_detector.color2(blurred, TO_YUV), self.val_field)
        field_cnts = self.field_detector.getContours(field_mask, RET_TREE)
        field_filled = self.field_detector.fill(frame, field_cnts)

        # Bitwise AND — hanya ambil area di dalam lapangan
        ball_frame = self.ball_detector.And(blurred, blurred, mask=field_filled)

        # HSV ball masking di dalam area lapangan
        ball_mask = self.ball_detector.mask(
            self.ball_detector.color2(ball_frame, TO_HSV), self.val_ball)
        ball_mask = self.ball_detector.morph(
            ball_mask, MORPH_CLOSE, self.ball_detector.getElement(ELE_ELLIPSE, 2))

        # Contour detection
        ball_cnts = self.ball_detector.getContours(ball_mask)
        ball_pos = self.ball_detector.circlePos(ball_cnts)

        # Fallback: blob detection
        if not ball_pos:
            blob_pos = self.ball_detector.blobPos(self.ball_detector.blob(~ball_mask))
            if blob_pos:
                ball_pos = blob_pos

        if ball_pos:
            self._ball_lost_count = 0
            cx = int(ball_pos["x_pos"])
            cy = int(ball_pos["y_pos"])
            size = int(ball_pos.get("size", 0))
            return cx, cy, size, size
        else:
            self._ball_lost_count += 1
            if self._ball_lost_count > self._BALL_LOST_THRESHOLD:
                self.ball_detector.kf.reset()
            return None, None, None, None

    def _detect_goal(self, frame):
        """HSV masking + contour + boundingRect untuk gawang."""
        # TODO: kalibrasi warna gawang via vision_calibrate.py
        blurred = VisionImage.blur(frame, sigma=5)
        goal_hsv = self.goal_detector.color2(blurred, TO_HSV)
        goal_mask = cv2.inRange(goal_hsv, np.array([20, 100, 100]), np.array([30, 255, 255]))

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        goal_mask = cv2.morphologyEx(goal_mask, cv2.MORPH_OPEN, kernel)
        goal_mask = cv2.morphologyEx(goal_mask, cv2.MORPH_CLOSE, kernel)

        contours = self.goal_detector.getContours(goal_mask)
        if len(contours) > 0:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > GOAL_MIN_AREA:
                x, y, w, h = cv2.boundingRect(largest)
                return x + w // 2, y + h // 2, w, h
        return None, None, None, None

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = cv2.resize(frame, (FRAME_W, FRAME_H))
            annotated = frame.copy()

            # Deteksi Bola
            ball_cx, ball_cy, ball_w, ball_h = self._detect_ball(frame)

            ball_bbox_msg = String()
            if ball_cx is not None:
                self.obj_pub.publish(String(data=f"{ball_cx},{ball_cy}"))
                ball_bbox_msg.data = f"{ball_cx},{ball_cy},{ball_w},{ball_h}"
                self.ball_detector.drawPoints(
                    annotated,
                    {"x_pos": ball_cx, "y_pos": ball_cy, "size": ball_w},
                    shape=DRAW_CIRCLE, label="Ball", disp_coordinates=True)
            else:
                ball_bbox_msg.data = ""
            self.ball_bbox_pub.publish(ball_bbox_msg)

            # Deteksi Gawang
            goal_cx, goal_cy, goal_w, goal_h = self._detect_goal(frame)

            goal_msg = String()
            if goal_cx is not None:
                goal_msg.data = f"{goal_cx},{goal_cy},{goal_w},{goal_h}"
                x1 = goal_cx - goal_w // 2
                y1 = goal_cy - goal_h // 2
                cv2.rectangle(annotated, (x1, y1), (x1 + goal_w, y1 + goal_h), (0, 255, 255), 2)
                cv2.putText(annotated, "Goal", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            else:
                goal_msg.data = ""
            self.goal_pub.publish(goal_msg)

            # FPS
            curr_time = time.time()
            if self.prev_time != 0:
                self.fps = 1.0 / (curr_time - self.prev_time)
            self.prev_time = curr_time
            cv2.putText(annotated, f"FPS: {self.fps:.1f}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            if self._show_gui:
                cv2.imshow("OpenCV Vision Node", annotated)
                cv2.waitKey(1)

        except (CvBridgeError, cv2.error, ValueError) as e:
            self.get_logger().error(f"Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = VisionCvNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
