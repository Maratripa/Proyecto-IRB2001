import numpy as np
import cv2 as cv
import sys
import threading

class Options:
    def __init__(self):
        super().__init__()
        self.masked_colors = [] # Colors to mask
        self.masks = [] # Masks
        self.frame = None
        self.hsv: cv.Mat = np.array([])

    def mouse_click(self, event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            if frame is not None:
                colors = frame[y, x]
                hsv = cv.cvtColor(np.uint8([[colors]]), cv.COLOR_BGR2HSV)
                self.masked_colors.append(hsv[0][0][0])

    def get_masks(self):
        self.masks = [cv.inRange(self.hsv, np.array([i - 10, 100, 50]), np.array([i + 10, 255, 255])) for i in self.masked_colors]
        if len(self.masked_colors) > 0:
            mask1 = cv.inRange(self.hsv, np.array([self.masked_colors[0] - 10, 100, 50]), np.array([self.masked_colors[0] + 10, 255, 255]))
            for color in self.masked_colors:
                mask2 = cv.inRange(self.hsv, np.array([color - 10, 100, 50]), np.array([color + 10, 255, 255]))
                mask1 = cv.bitwise_or(mask1, mask2)

            return mask1

    def get_centers(self) -> list:
        if len(self.masked_colors) > 0:
            centers = []
            for mask in self.masks:
                bilateral = cv.bilateralFilter(mask, 9, 75, 75)
                median = cv.medianBlur(bilateral, 7)
                M = cv.moments(median)
                if M["m00"] != 0:
                    cX = int(M["m10"] / (M["m00"]))
                    cY = int(M["m01"] / (M["m00"]))
                    centers.append((cX, cY))

            return centers
        else:
            return []

class Data:
    def __init__(self):
        self.angulo = 0.0
        self.dist_1 = 0.0
        self.dist_2 = 0.0

    def start_thread(self):
        thread = threading.Thread(target=self.send_data, name="sender", daemon=True, args=())
        thread.start()

    def send_data(self):
        print(self.angulo, self.dist_1, self.dist_2)

if __name__ == "__main__":
    cap = cv.VideoCapture(int(sys.argv[1]))
    options = Options()
    data = Data()
    data.start_thread()

    font = cv.FONT_HERSHEY_SIMPLEX

    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    cv.namedWindow('frame')
    cv.setMouseCallback('frame', options.mouse_click)

    while True:
        # Capture frame-by-frame
        ret, options.frame = cap.read()
        frame = options.frame
        options.hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        if len(options.masked_colors) > 0:
            mask = options.get_masks()
            result = cv.bitwise_and(frame, frame, mask=mask)
        else:
            result = frame

        centers = options.get_centers()
        for point in centers:
            cv.circle(result, point, 5, (0, 0, 255), -1)

        """ guia centros
        [0] -> adelante auto 1
        [1] -> atras auto 1
        [2] -> pelota
        [3] -> adelante auto 2
        [4] -> atras auto 2
        """
        if len(centers) > 1:
            cv.line(result, centers[0], centers[1], (0, 255, 0), 2) # adelante / atras auto 1
            if len(centers) > 2:
                cv.line(result, centers[0], centers[2], (0, 255, 0), 2) # adelante a1 / pelota

                v1 = np.array(centers[1]) - np.array(centers[0]) # atras adelante
                # v1_1 = np.array(centers[2] - np.array(centers[1])) # pelota atras
                v2 = np.array(centers[2]) - np.array(centers[0]) # pelota adelante

                norm_1 = np.linalg.norm(v1)
                # norm_1_1 = np.linalg.norm(v1_1)
                norm_2 = np.linalg.norm(v2)

                # angle = np.arccos((norm_1**2 + norm_2**2 - norm_1_1**2) / (2 * norm_1 * norm_2))

                atan1 = np.arctan2(v1[1], v1[0]) # a atras
                atan2 = np.arctan2(v2[1], v2[0]) # a pelota

                angulo = atan2 - atan1
                if angulo > np.pi:
                    angulo -= 2 * np.pi
                elif angulo < -np.pi:
                    angulo += 2 * np.pi

                ang = angulo * 180 / np.pi
                data.angulo = float(ang)
                data.dist_1 = float(norm_2)

                cv.putText(result, f"Angulo A1 | pelota: {ang}°", (10, 10), font, .3, (255, 255, 255), 1, cv.LINE_AA)
                cv.putText(result, f"d1 adelante a1 | pelota: {norm_2} px", (10, 20), font, .3, (255, 255, 255), 1, cv.LINE_AA)

                if len(centers) > 4:
                    cv.line(result, centers[3], centers[0], (0, 255, 0), 2)
                    cv.line(result, centers[3], centers[4], (0, 255, 0), 2)

                    v3 = np.array(centers[0]) - np.array(centers[3])
                    norm_3 = np.linalg.norm(v3)

                    data.dist_2 = float(norm_3)

                    cv.putText(result, f"d2 adelante a1 | adelante a1: {norm_3} px", (10, 30), font, .3, (255, 255, 255), 1, cv.LINE_AA)


        # Display the resulting frame
        cv.imshow('frame', frame)
        cv.imshow('result', result)

        k = cv.waitKey(1)
        if k == ord('q'):
            break
        elif k == ord('u'):
            options.masked_colors.pop()

    # When everything done, release the capture
    cap.release()
    cv.destroyAllWindows()