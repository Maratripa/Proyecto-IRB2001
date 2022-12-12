import processor as proc
from parametros import K1_D, K1_I, K2_D, K2_I
from robot import Robot
import threading
import serial
import time
import sys

class SerialCom():
    def __init__(self, port, baudrate, timeout, robot):
        try:
            self.serial = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
            print("Conexion establecida!")
        except:
            print("No se ha establecido conexion...")
            self.serial = None

        self.robot = robot

    def start_data_thread(self):
        thread = threading.Thread(target=self.send_data, args=(), daemon=True)
        thread.start()

    def send_data(self):
        while self.serial is not None:
            if self.robot.data != {}:
                m0, m1 = self.robot.move_to_obj()

                self.serial.write(str.encode(f"{m0}:{m1};"))

                time.sleep(0.06)

def main(src):
    capture = proc.VideoCapture(src).start()
    processor = proc.ProcessMasks(capture.frame, capture.masked_colors).start()

    com = SerialCom("/dev/rfcomm0", 38400, 5, Robot())
    com.start_data_thread()

    while True:
        if capture.stopped or capture.stopped:
            capture.stop()
            processor.stop()
            break

        processor.set_objective(capture.state)
        com.robot.state = capture.state
        if len(processor.arcos) < 2:
            processor.arcos = capture.arcos
        
        processor.frame = capture.frame
        processor.masked_colors = capture.masked_colors
        
        capture.centers = processor.centers
        capture.mask = processor.get_joint_masks() # type: ignore

        com.robot.data = processor.data



if __name__ == "__main__":
    source = int(sys.argv[1])
    main(source)