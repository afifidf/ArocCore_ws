# vision_calibrate.py — Tool kalibrasi HSV bola + lapangan sekaligus
# Jalankan: ros2 run headControl26 calibrate
# Atau via launch: ros2 launch headControl26 headControl26.launch.py

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np
import ast

FRAME_W = 640
FRAME_H = 480
CALIB_DIR = os.path.dirname(os.path.abspath(__file__))


class CalibrateNode(Node):
    def __init__(self):
        super().__init__('calibrate_node')

        self.bridge = CvBridge()
        self.frame = None

        # Load nilai lama
        self.ball_lower, self.ball_upper = self._load("robotis_orange_ball", [3, 68, 21], [33, 255, 243])
        self.field_lower, self.field_upper = self._load("robotis_green_field", [65, 75, 59], [230, 120, 208])

        # Trackbar — Ball
        cv2.namedWindow("Ball")
        cv2.createTrackbar("Low H", "Ball", self.ball_lower[0], 255, lambda x: None)
        cv2.createTrackbar("Low S", "Ball", self.ball_lower[1], 255, lambda x: None)
        cv2.createTrackbar("Low V", "Ball", self.ball_lower[2], 255, lambda x: None)
        cv2.createTrackbar("Max H", "Ball", self.ball_upper[0], 255, lambda x: None)
        cv2.createTrackbar("Max S", "Ball", self.ball_upper[1], 255, lambda x: None)
        cv2.createTrackbar("Max V", "Ball", self.ball_upper[2], 255, lambda x: None)

        # Trackbar — Field
        cv2.namedWindow("Field")
        cv2.createTrackbar("Low H", "Field", self.field_lower[0], 255, lambda x: None)
        cv2.createTrackbar("Low S", "Field", self.field_lower[1], 255, lambda x: None)
        cv2.createTrackbar("Low V", "Field", self.field_lower[2], 255, lambda x: None)
        cv2.createTrackbar("Max H", "Field", self.field_upper[0], 255, lambda x: None)
        cv2.createTrackbar("Max S", "Field", self.field_upper[1], 255, lambda x: None)
        cv2.createTrackbar("Max V", "Field", self.field_upper[2], 255, lambda x: None)

        self.create_subscription(
            Image, '/image_raw',
            self._image_callback, qos_profile_sensor_data)

        self.create_timer(0.033, self._update)

        self.get_logger().info("✅ Calibrate Node — Ball + Field (tekan 's' save, 'q' keluar)")

    def _load(self, name, default_lower, default_upper):
        filepath = os.path.join(CALIB_DIR, name + ".txt")
        try:
            with open(filepath, "r") as f:
                d = ast.literal_eval(f.read())
                lower = d.get(name + "_lower_val", default_lower)
                upper = d.get(name + "_upper_val", default_upper)
                self.get_logger().info(f"Loaded {name}: {lower} → {upper}")
                return lower, upper
        except (OSError, SyntaxError, ValueError) as e:
            self.get_logger().warn(f"Failed to load {name}: {e}. Using defaults.")
            return default_lower, default_upper

    def _save(self, name, lower, upper):
        filepath = os.path.join(CALIB_DIR, name + ".txt")
        val = {name + "_lower_val": lower, name + "_upper_val": upper}
        with open(filepath, "w") as f:
            f.write(str(val))
        self.get_logger().info(f"💾 Saved {name}: {val}")

    def _read_trackbar(self, win):
        lh = cv2.getTrackbarPos("Low H", win)
        ls = cv2.getTrackbarPos("Low S", win)
        lv = cv2.getTrackbarPos("Low V", win)
        uh = cv2.getTrackbarPos("Max H", win)
        us = cv2.getTrackbarPos("Max S", win)
        uv = cv2.getTrackbarPos("Max V", win)
        return np.array([lh, ls, lv]), np.array([uh, us, uv])

    def _image_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame = cv2.resize(self.frame, (FRAME_W, FRAME_H))
        except (CvBridgeError, cv2.error, ValueError) as e:
            self.get_logger().error(f"Error: {e}")

    def _update(self):
        if self.frame is None:
            return

        blurred = cv2.GaussianBlur(self.frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        yuv = cv2.cvtColor(blurred, cv2.COLOR_BGR2YUV)

        # Ball mask (HSV)
        b_low, b_up = self._read_trackbar("Ball")
        ball_mask = cv2.inRange(hsv, b_low, b_up)
        ball_result = cv2.bitwise_and(self.frame, self.frame, mask=ball_mask)

        # Field mask (YUV)
        f_low, f_up = self._read_trackbar("Field")
        field_mask = cv2.inRange(yuv, f_low, f_up)
        field_result = cv2.bitwise_and(self.frame, self.frame, mask=field_mask)

        cv2.imshow("Original", self.frame)
        cv2.imshow("Ball Mask", ball_mask)
        cv2.imshow("Ball Result", ball_result)
        cv2.imshow("Field Mask", field_mask)
        cv2.imshow("Field Result", field_result)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            self._save("robotis_orange_ball", b_low.tolist(), b_up.tolist())
            self._save("robotis_green_field", f_low.tolist(), f_up.tolist())
        elif key == ord('q'):
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
