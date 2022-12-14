import processor as proc
import threading
import serial
import time
import sys

class SerialCom():
    def __init__(self, port, baudrate, timeout):
        try:
            self.serial = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
            print("Conexion establecida!")
        except:
            print("No se ha establecido conexion...")
            self.serial = None

        self.data = {}

    def start_data_thread(self):
        thread = threading.Thread(target=self.send_data, args=(), daemon=True)
        thread.start()

    def send_data(self):
        while self.serial is not None:
            if self.data != {}:
                m0 = 1 * self.data['d1'] + 10 * self.data['a']
                m1 = 1 * self.data['d1'] + 10 * self.data['a']

                self.serial.write(str.encode(f"{m0}:{m1};"))

                print(f"Angulo: {self.data['a']}\t|\tD1: {self.data['d1']}\t|\tD2: {self.data['d2']}")

                time.sleep(0.01)

def main(src):
    capture = proc.VideoCapture(src).start()
    processor = proc.ProcessMasks(capture.frame, capture.masked_colors).start()

    com = SerialCom("COM10", 38400, 1)
    com.start_data_thread()

    while True:
        if capture.stopped or processor.stopped:
            capture.stop()
            processor.stop()
            break

        processor.frame = capture.frame
        processor.masked_colors = capture.masked_colors
        capture.centers = processor.centers
        capture.mask = processor.get_joint_masks() # type:ignore

        com.data = processor.data

if __name__ == "__main__":
    source = int(sys.argv[1])
    main(source)