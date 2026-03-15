import numpy as np
import cv2


class VisionImage:
    """Utility class untuk pemrosesan gambar."""

    @staticmethod
    def resize(image, width=None, height=None, interpolation=cv2.INTER_AREA):
        h, w = image.shape[:2]
        if width is None and height is None:
            return image
        if width is None:
            r = height / float(h)
            dim = (int(w * r), height)
        else:
            r = width / float(w)
            dim = (width, int(h * r))
        return cv2.resize(image, dim, interpolation=interpolation)

    @staticmethod
    def blur(frame, sigma=11):
        return cv2.GaussianBlur(frame, (sigma, sigma), 0)

    @staticmethod
    def set_brightness(frame, value):
        h, s, v = cv2.split(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV))
        v = np.clip(v.astype(int) + value, 0, 255).astype(np.uint8)
        return cv2.cvtColor(cv2.merge((h, s, v)), cv2.COLOR_HSV2BGR)

    @staticmethod
    def set_contrast(frame, value):
        alpha = float(131 * (value + 127)) / (127 * (131 - value))
        gamma = 127 * (1 - alpha)
        return cv2.addWeighted(frame, alpha, frame, 0, gamma)

    @staticmethod
    def set_brightness_contrast(frame, bright=0.0, contr=0.0, beta=0.0):
        return cv2.addWeighted(frame, 1 + float(contr) / 100.0, frame, beta, float(bright))
