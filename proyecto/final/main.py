import processor as proc
from parametros import K1_D, K1_I, K2_D, K2_I
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
                m0 = K1_I * self.data['d1'] + K2_I * self.data['a']
                m1 = K1_D * self.data['d1'] + K2_D * self.data['a']

                self.serial.write(str.encode(f"{m0}:{m1};"))

                print(f"Angulo: {self.data['a']}\t|\tD1: {self.data['d1']}\t|\tD2: {self.data['d2']}")

                time.sleep(0.01)

def main(src):
    capture = proc.VideoCapture(src).start()
    processor = proc.ProcessMasks(capture.frame, capture.masked_colors).start()

    com = SerialCom("/dev/rfcomm0", 38400, 5)
    com.start_data_thread()

    while True:
        if capture.stopped or capture.stopped:
            capture.stop()
            processor.stop()
            break
        
        processor.frame = capture.frame
        processor.masked_colors = capture.masked_colors
        processor.set_objective(capture.objective)
        capture.centers = processor.centers
        # capture.mask = processor.get_joint_masks() # type: ignore

        com.data = processor.data

if __name__ == "__main__":
    source = int(sys.argv[1])
    main(source)