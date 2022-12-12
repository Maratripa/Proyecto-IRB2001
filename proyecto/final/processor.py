"""
https://nrsyed.com/20183/07/05/multithreading-with-opencv-python-to-improve-video-processing-performance/
"""

import cv2
import threading
import numpy as np
import time

class VideoCapture:
    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

        self.masked_colors = []
        self.mask = None
        self.centers = []

        self.arcos = []

        self.screen_center = np.array(self.frame.shape[:2][::-1]) // 2
        # # print(self.screen_center)
        self.state = "center"

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.frame is not None and len(self.masked_colors) < 5:
                colors = self.frame[y, x]
                hsv = cv2.cvtColor(np.array([[colors]], dtype=np.uint8), cv2.COLOR_BGR2HSV)
                self.masked_colors.append(hsv[0][0])
                print("Color added to masked_colors")
            elif self.frame is not None and len(self.masked_colors) >= 5 and len(self.arcos) < 2:
                self.arcos.append(np.array([x, y]))
    
    def get(self):
        if not self.grabbed:
            self.stop()
        (self.grabbed, self.frame) = self.stream.read()
    
    def show(self):
        if self.mask is not None:
            result = cv2.bitwise_and(self.frame, self.frame, mask=self.mask) # type: ignore
        else:
            result = self.frame
        frame = cv2.bitwise_and(self.frame, self.frame)

        for point in self.centers:
            cv2.circle(frame, point, 6, (255, 255, 255), -1) # type: ignore
        
        # if self.objective == self.screen_center:
        #     cv2.circle(result, self.objective, 6, (255, 255, 255), -1)
        for arco in self.arcos:
            cv2.circle(frame, arco, 5, (255, 0, 0), -1)

        # Draw centers and lines
        if len(self.centers) > 2:
            if self.state == "center":
                objetivo = self.screen_center
            else:
                objetivo = self.centers[2]
            
            cv2.line(frame, self.centers[0], self.centers[1], (255, 255, 255), 3) #type: ignore , linea adelante atras
            cv2.line(frame, self.centers[1], objetivo, (255, 255, 255), 3) #type: ignore , linea atras objetivo

        cv2.imshow("Video", frame)
        cv2.imshow("Mask", result)
    
    def start(self):
        thread = threading.Thread(target=self.main, args=(), daemon=True)
        thread.start()
        return self
    
    def main(self):
        cv2.namedWindow("Video")
        cv2.setMouseCallback("Video", self.mouse_callback)
        while True:
            self.get()
            self.show()

            k = cv2.waitKey(1)
            if k == ord('q'):
                self.stop()
            elif k == ord('u'):
                if len(self.masked_colors) > 0:
                    self.masked_colors.pop()
            elif k == ord('c'):
                self.state = "center"
            elif k == ord('b'):
                self.state = "ball"
            elif k == ord('n'):
                self.state = "no tocar"
            elif k == ord('l'):
                self.state = "line"
            elif k == ord('d'):
                self.state = "def_1"
            elif k == ord('f'):
                self.state = "def_2"
            elif k == ord('g'):
                self.state = "def_3"

    def stop(self):
        self.stopped = True

class ProcessMasks:
    """Class that gets masks, centers and data for post-processing using a different thread"""
    def __init__(self, frame=np.zeros((480, 640, 3)), masked_colors=[]):
        self.__frame = frame
        self.masked_colors = masked_colors
        self.masks = []

        self.stopped = False

        self.data = {
            'd0': 0.0,
            'a': 0.0,
            'd1': 0.0,
            'd2': 0.0,
            'd3': 0.0,
            'd4': 0.0,
            'a3': 0.0,
            'a4': 0.0
        }

        self.centers = []
        self.screen_center = np.array(self.frame.shape[:2][::-1]) // 2
        self.objective = self.screen_center

        self.arcos = []

    @property
    def frame(self):
        return self.__frame

    @frame.setter
    def frame(self, value):
        self.__frame = value
        self.get_masks()
        self.centers = self.get_centers()
    
    def set_objective(self, obj: str):
        if obj == "center":
            self.objective = self.screen_center
        elif obj in ("ball", "no tocar", "line", "def_1", "def_3") and len(self.centers) > 2:
            self.objective = self.centers[2]
        elif obj == "def_2" and len(self.centers) > 2 and len(self.arcos) > 0:
            self.objective = (self.centers[2] + self.arcos[0]) // 2
        else:
            self.objective = self.screen_center

    def get_joint_masks(self):
            hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV) # type: ignore
            delta = np.array([10, 40, 40])
            if len(self.masked_colors) > 0:
                mask1 = cv2.inRange(hsv, self.masked_colors[0] - delta, self.masked_colors[0] + delta)
                if len(self.masked_colors) > 1:
                    for color in self.masked_colors[1:]:
                        mask2 = cv2.inRange(hsv, color - delta, color + delta)
                        mask1 = cv2.bitwise_or(mask1, mask2)

                return mask1

    def get_masks(self):
        if self.frame is not None:
            delta = np.array([10, 40, 40])
            hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)  # type: ignore
            self.masks = [cv2.inRange(hsv, i - delta, i + delta) for i in self.masked_colors]

    def get_centers(self) -> list:
        if len(self.masks) > 0:
            centers = []
            for mask in self.masks:
                # bilateral = cv2.bilateralFilter(mask, 9, 75, 75)
                median = cv2.medianBlur(mask, 7)
                blur = cv2.GaussianBlur(median, (5, 5), 0)
                _, th3 = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                M = cv2.moments(th3)
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
                v2 = np.array(self.objective) - np.array(self.centers[1]) # objetivo - atras

                norm_1 = np.linalg.norm(v1)
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
                self.data['d0'] = float(norm_1)
                self.data['d1'] = float(norm_2)

                if len(self.centers) > 4:
                    v3 = np.array(self.centers[0]) - np.array(self.centers[3])
                    norm_3 = np.linalg.norm(v3)
                    self.data['d2'] = float(norm_3)
                
                for i, arco in enumerate(self.arcos):
                    v = arco - np.array(self.centers[1])

                    atan_2 = np.arctan2(v[1], v[0])

                    ang = atan_2 - atan1
                    norm = np.linalg.norm(v)
                    self.data[f"d{3+i}"] = float(norm)
                    self.data[f"a{3+i}"] = float(ang)
                
                time.sleep(0.001)

    def stop(self):
        self.stopped = True

if __name__ == "__main__":
    capture = VideoCapture(0).start()
    processor = ProcessMasks(capture.frame, capture.masked_colors).start()

    while True:
        if capture.stopped or processor.stopped:
            capture.stop()
            processor.stop()
            break

        processor.frame = capture.frame
        processor.masked_colors = capture.masked_colors
        capture.centers = processor.centers
        # capture.mask = processor.get_joint_masks() # type: ignore