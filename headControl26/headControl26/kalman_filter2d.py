import cv2
import numpy as np


class KalmanFilter2D:
    """Kalman Filter 2D (state: [x, y, vx, vy], measurement: [x, y])."""

    def __init__(self):
        self.kalman = cv2.KalmanFilter(4, 2)

        self.kalman.transitionMatrix = np.array(
            [[1, 0, 1, 0],
             [0, 1, 0, 1],
             [0, 0, 1, 0],
             [0, 0, 0, 1]], np.float32)

        self.kalman.measurementMatrix = np.array(
            [[1, 0, 0, 0],
             [0, 1, 0, 0]], np.float32)

        self.kalman.processNoiseCov = np.array(
            [[1, 0, 0, 0],
             [0, 1, 0, 0],
             [0, 0, 2, 0],
             [0, 0, 0, 2]], np.float32) * 0.1

        self.kalman.measurementNoiseCov = np.array(
            [[1, 0],
             [0, 1]], np.float32) * 1.0

        self.kalman.errorCovPost = np.eye(4, dtype=np.float32) * 10
        self.kalman.statePost = np.zeros((4, 1), dtype=np.float32)
        self._initialized = False

    def update(self, cx, cy):
        """Update KF dengan pengukuran baru dan return (smoothed_x, smoothed_y)."""
        measurement = np.array([[np.float32(cx)], [np.float32(cy)]])

        if not self._initialized:
            self.kalman.statePost = np.array(
                [[np.float32(cx)], [np.float32(cy)], [0], [0]], np.float32)
            self._initialized = True

        self.kalman.correct(measurement)
        prediction = self.kalman.predict()
        return float(prediction[0]), float(prediction[1])

    def reset(self):
        self.kalman.statePost = np.zeros((4, 1), dtype=np.float32)
        self.kalman.errorCovPost = np.eye(4, dtype=np.float32) * 10
        self._initialized = False
