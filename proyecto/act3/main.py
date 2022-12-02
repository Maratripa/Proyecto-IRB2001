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
    video_getter = proc.VideoGet(src).start()
    video_shower = proc.VideoShow(video_getter.frame).start()
    processor = proc.ProcessMasks(video_getter.frame, video_shower.masked_colors).start()

    com = SerialCom("COM10", 38400, 1)
    com.start_data_thread()

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

        com.data = processor.data
        time.sleep(0.01)

if __name__ == "__main__":
    source = int(sys.argv[1])
    main(source)