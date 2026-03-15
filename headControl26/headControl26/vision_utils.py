import numpy as np
import cv2
import ast
from .kalman_filter2d import KalmanFilter2D

TO_HSV = cv2.COLOR_BGR2HSV
TO_YUV = cv2.COLOR_BGR2YUV
TO_LAB = cv2.COLOR_BGR2LAB

ELE_NORM = 0
ELE_RECT = cv2.MORPH_RECT
ELE_CROSS = cv2.MORPH_CROSS
ELE_ELLIPSE = cv2.MORPH_ELLIPSE

MORPH_OPEN = cv2.MORPH_OPEN
MORPH_CLOSE = cv2.MORPH_CLOSE
MORPH_GRADIENT = cv2.MORPH_GRADIENT

RET_EXT = cv2.RETR_EXTERNAL
RET_TREE = cv2.RETR_TREE

DRAW_RECT = 0
DRAW_CIRCLE = 2


class Contours:
    def __init__(self):
        self.kf = KalmanFilter2D()

    def getContours(self, filtered_frame, method=cv2.RETR_EXTERNAL):
        return cv2.findContours(filtered_frame, method, cv2.CHAIN_APPROX_SIMPLE)[-2]

    def fill(self, size_frame, contours):
        """Convex hull fill — untuk green field masking."""
        hull = []
        drawing = np.zeros((size_frame.shape[0], size_frame.shape[1], 1), np.uint8)
        for i in range(len(contours)):
            hull.append(cv2.convexHull(contours[i], False))
            cv2.drawContours(drawing, hull, i, (255, 255, 255), -1, 8)
        drawing = cv2.dilate(drawing, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2)))
        drawing = cv2.dilate(drawing, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2)))
        drawing = cv2.morphologyEx(drawing, cv2.MORPH_OPEN,
                                   cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
        return drawing

    def circlePos(self, cnts):
        """Return dict {x_pos, y_pos, size} dari contour terbesar + KF smoothing."""
        pos = {}
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            try:
                kf_x, kf_y = self.kf.update(x, y)
                pos = {"x_pos": kf_x, "y_pos": kf_y, "size": radius * 2}
            except ZeroDivisionError:
                pass
        return pos

    def rectPos(self, cnts):
        """Return dict {x_pos, y_pos, width, height} dari contour terbesar."""
        pos = {}
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
            pos = {"x_pos": x, "y_pos": y, "width": w, "height": h}
        return pos


class Drawing:
    def drawPoints(self, frame, pos, shape=DRAW_CIRCLE, label="obj", disp_coordinates=False):
        if shape == DRAW_CIRCLE:
            try:
                if pos.get("size", 0) > 0:
                    cv2.circle(frame, (int(pos["x_pos"]), int(pos["y_pos"])),
                               int(pos["size"] / 2), (0, 255, 0), 2)
                    cv2.circle(frame, (int(pos["x_pos"]), int(pos["y_pos"])),
                               3, (0, 0, 255), -1)
                    if disp_coordinates:
                        cv2.putText(frame, label,
                                    (int(pos["x_pos"]) + 10, int(pos["y_pos"])),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
                        cv2.putText(frame,
                                    f"({int(pos['x_pos'])},{int(pos['y_pos'])})",
                                    (int(pos["x_pos"]) + 10, int(pos["y_pos"]) + 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
            except KeyError:
                pass
        elif shape == DRAW_RECT:
            try:
                x, y = int(pos["x_pos"]), int(pos["y_pos"])
                w, h = int(pos["width"]), int(pos["height"])
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                if disp_coordinates:
                    cv2.putText(frame, label, (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            except KeyError:
                pass

    @staticmethod
    def enableReferenceLine(frame):
        h, w = frame.shape[:2]
        # Tilt
        cv2.line(frame, (0, h // 2 - 20), (w, h // 2 - 20), (255, 0, 255), 2)
        cv2.line(frame, (0, h // 2), (w, h // 2), (0, 0, 255), 2)
        cv2.line(frame, (0, h // 2 + 20), (w, h // 2 + 20), (255, 0, 255), 2)
        cv2.line(frame, (0, h - 85), (w, h - 85), (255, 0, 0), 2)
        cv2.line(frame, (0, 85), (w, 85), (255, 0, 0), 2)
        # Pan
        cv2.line(frame, (100, 0), (100, h), (255, 120, 120), 2)
        cv2.line(frame, (w - 100, 0), (w - 100, h), (255, 120, 120), 2)
        cv2.line(frame, (w // 2 - 20, 0), (w // 2 - 20, h), (0, 0, 255), 2)
        cv2.line(frame, (w // 2, 0), (w // 2, h), (0, 0, 255), 2)
        cv2.line(frame, (w // 2 + 20, 0), (w // 2 + 20, h), (0, 0, 255), 2)


class Filter(Contours, Drawing):
    def getElement(self, elem_type, size):
        if not elem_type:
            return np.ones((size, size), np.uint8)
        return cv2.getStructuringElement(elem_type, (size, size))

    def color2(self, frame, color_type):
        return cv2.cvtColor(frame, color_type)

    def morph(self, frame, morph_type, kernel, iterate=1):
        return cv2.morphologyEx(frame, morph_type, kernel, iterations=iterate)


class Bitwise:
    def And(self, src1, src2, mask=None):
        return cv2.bitwise_and(src1, src2, mask=mask)

    def Or(self, src1, src2, mask=None):
        return cv2.bitwise_or(src1, src2, mask=mask)

    def Not(self, src, mask=None):
        return cv2.bitwise_not(src, mask=mask)


class Blob:
    def __init__(self):
        self.params = None

    def blobSetParams(self, params=None):
        if params is not None:
            self.params = params
        else:
            self.params = cv2.SimpleBlobDetector_Params()
            self.params.minThreshold = 0
            self.params.maxThreshold = 100
            self.params.filterByArea = True
            self.params.minArea = 200
            self.params.maxArea = 40000
            self.params.filterByCircularity = True
            self.params.minCircularity = 0.1
            self.params.filterByConvexity = True
            self.params.minConvexity = 0.5
            self.params.filterByInertia = True
            self.params.minInertiaRatio = 0.5

    def blob(self, filtered_frame):
        detector = cv2.SimpleBlobDetector_create(self.params)
        return detector.detect(filtered_frame)

    def blobPos(self, keypoints):
        """Return dict {x_pos, y_pos, size} dari keypoint terbesar."""
        if not keypoints:
            return {}
        kp = max(keypoints, key=lambda point: point.size)
        return {"x_pos": kp.pt[0], "y_pos": kp.pt[1], "size": kp.size}


class ColorBased(Filter, Bitwise, Blob):
    """Class utama untuk deteksi objek berbasis warna HSV."""

    def __init__(self):
        Blob.__init__(self)
        Contours.__init__(self)
        self.__isUsingTrackbar = False
        self.__windowName = "tracking"
        self.__trackName = ["L", "L", "L", "U", "U", "U"]

    def createTrackbar(self, winName=None, trackName=None):
        if winName is not None and trackName is not None:
            self.__windowName = winName
            if len(trackName) == 3:
                for i, dat in enumerate(trackName):
                    self.__trackName[i] += dat
                    self.__trackName[i + 3] += dat

        cv2.namedWindow(self.__windowName)
        try:
            for index, data in enumerate(self.__trackName):
                if index < len(self.__trackName) / 2:
                    cv2.createTrackbar(data, self.__windowName, 0, 255, self._nothing)
                else:
                    cv2.createTrackbar(data, self.__windowName, 255, 255, self._nothing)
        except TypeError:
            pass
        self.__isUsingTrackbar = True

    def calibrate(self, filtered_frame, path="value"):
        """Baca trackbar → simpan ke file → return mask."""
        if self.__isUsingTrackbar:
            lower, upper = [], []
            for index, data in enumerate(self.__trackName):
                if index < len(self.__trackName) / 2:
                    lower.append(cv2.getTrackbarPos(data, self.__windowName))
                else:
                    upper.append(cv2.getTrackbarPos(data, self.__windowName))
            val = {path + "_lower_val": lower, path + "_upper_val": upper}
            with open(path + ".txt", "w") as f:
                f.write(str(val))
            return cv2.inRange(filtered_frame, np.array(lower), np.array(upper))

    def load(self, path):
        """Load nilai HSV lower/upper dari file .txt dan return [lower, upper]."""
        with open(path + ".txt", "r") as file:
            d = ast.literal_eval(file.read())
            return [d[path + "_lower_val"], d[path + "_upper_val"]]

    def mask(self, filtered_frame, value):
        """Apply inRange dengan value = [lower, upper] dari load()."""
        return cv2.inRange(filtered_frame, np.array(value[0]), np.array(value[1]))

    def _nothing(self, x):
        pass
