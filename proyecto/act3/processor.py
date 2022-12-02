"""
https://nrsyed.com/20183/07/05/multithreading-with-opencv-python-to-improve-video-processing-performance/
"""

import cv2
import threading
import numpy as np
import numba as nb
import time

class VideoGet:
    """Get frame with dedicated thread"""
    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        thread = threading.Thread(target=self.get, args=(), daemon=True)
        thread.start()
        return self

    def get(self):
        while not self.stopped:
            if not self.grabbed:
                self.stop()

            (self.grabbed, self.frame) = self.stream.read()
            time.sleep(0.01)

    def stop(self):
        self.stopped = True

class VideoShow:
    """Class that continuously shows a frame using a different thread"""
    def __init__(self, frame=None, mask=None):
        self.frame = frame
        self.mask = mask
        self.stopped = False
        self.masked_colors = []
        self.centers = []

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.frame is not None:
                colors = self.frame[y, x]
                hsv = cv2.cvtColor(np.array([[colors]], dtype=np.uint8), cv2.COLOR_BGR2HSV)
                self.masked_colors.append(hsv[0][0][0])
                print("Color added to masked_colors")

    def start(self):
        thread = threading.Thread(target=self.show, args=(), daemon=True)
        thread.start()
        return self

    def show(self):
        cv2.namedWindow("Video")
        cv2.setMouseCallback("Video", self.mouse_callback)
        while not self.stopped:
            if self.mask is not None:
                result = cv2.bitwise_and(self.frame, self.frame, mask=self.mask) # type: ignore
            else:
                result = self.frame

            # Draw centers and lines
            if len(self.centers) > 2:
                for point in self.centers:
                    cv2.circle(result, point, 5, (255, 0, 0), -1) # type: ignore

                cv2.line(result, self.centers[0], self.centers[1], (0, 255, 0), 3) #type: ignore , linea adelante atras
                cv2.line(result, self.centers[1], self.centers[2], (0, 0, 255), 3) #type: ignore , linea atras pelota

            cv2.imshow("Video", self.frame)
            cv2.imshow("Masks", result)

            k = cv2.waitKey(1)
            if k == ord('q'):
                self.stop()
            elif k == ord('u'):
                self.masked_colors.pop()
            time.sleep(0.01)

    def stop(self):
        self.stopped = True

class ProcessMasks:
    """Class that gets masks, centers and data for post-processing using a different thread"""
    def __init__(self, frame=None, masked_colors=[]):
        self.__frame = frame
        self.masked_colors = masked_colors
        self.masks = []

        self.stopped = False

        self.data = {
            'a': 0.0,
            'd1': 0.0,
            'd2': 0.0
        }

        self.centers = []

    @property
    def frame(self):
        return self.__frame

    @frame.setter
    def frame(self, value):
        self.__frame = value
        self.get_masks()
        self.centers = self.get_centers()

    def get_joint_masks(self):
            hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV) # type: ignore
            if len(self.masked_colors) > 0:
                mask1 = cv2.inRange(hsv, np.array([self.masked_colors[0] - 10, 100, 50]),
                    np.array([self.masked_colors[0] + 10, 255, 255]))
                if len(self.masked_colors) > 1:
                    for color in self.masked_colors[1:]:
                        mask2 = cv2.inRange(hsv, np.array([color - 10, 100, 50]),
                            np.array([color + 10, 255, 255]))
                        mask1 = cv2.bitwise_or(mask1, mask2)

                return mask1

    # @nb.njit
    def get_masks(self):
        if self.frame is not None:
            hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)  # type: ignore
            self.masks = [cv2.inRange(hsv, np.array([i - 10, 100, 50]),
                np.array([i + 10, 255, 255])) for i in self.masked_colors]

    # @nb.njit
    def get_centers(self) -> list:
        if len(self.masks) > 0:
            centers = []
            for mask in self.masks:
                bilateral = cv2.bilateralFilter(mask, 9, 75, 75)
                median = cv2.medianBlur(bilateral, 7)
                M = cv2.moments(median)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    centers.append((cx, cy))
            return centers
        return []

    def start(self):
        thread_main = threading.Thread(target=self.compute, args=(), daemon=True)
        thread_main.start()

        return self

    # @nb.njit
    def compute(self):
        while not self.stopped:
            """ guia centros
            [0] -> adelante auto 1
            [1] -> atras auto 1
            [2] -> pelota
            [3] -> adelante auto 2
            [4] -> atras auto 2
            """
            if len(self.centers) > 2:
                v1 = np.array(self.centers[0]) - np.array(self.centers[1]) # adelante - atras
                v2 = np.array(self.centers[2]) - np.array(self.centers[1]) # pelota - atras

                # norm_1 = np.linalg.norm(v1)
                norm_2 = np.linalg.norm(v2)

                atan1 = np.arctan2(v1[1], v1[0]) # a atras
                atan2 = np.arctan2(v2[1], v2[0]) # a pelota

                angulo = atan2 - atan1
                if angulo > np.pi:
                    angulo -= 2 * np.pi
                elif angulo < -np.pi:
                    angulo += 2 * np.pi

                angulo = angulo * 180 / np.pi
                self.data['a'] = float(angulo)
                self.data['d1'] = float(norm_2)

                if len(self.centers) > 4:
                    v3 = np.array(self.centers[0]) - np.array(self.centers[3])
                    norm_3 = np.linalg.norm(v3)
                    self.data['d2'] = float(norm_3)

    def stop(self):
        self.stopped = True

if __name__ == "__main__":
    video_getter = VideoGet(0).start()
    video_shower = VideoShow(video_getter.frame).start()
    processor = ProcessMasks(video_getter.frame, video_shower.masked_colors).start()

    while True:
        if video_getter.stopped or video_shower.stopped:
            video_shower.stop()
            video_getter.stop()
            processor.stop()
            break

        frame = video_getter.frame
        processor.frame = frame

        video_shower.frame = frame
        video_shower.centers = processor.centers

        processor.masked_colors = video_shower.masked_colors
        video_shower.mask = processor.get_joint_masks()

        time.sleep(0.01)