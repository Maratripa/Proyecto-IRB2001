import numpy as np
import time
import serial

# Mensaje que quiero enviar
msg_on = "y"

# Mensaje tiene que ser encodeado
msg_enc = str.encode(msg_on)

# serial.Serial nos permite abrir el puerto COM deseado
ser = serial.Serial("/dev/rfcomm0", baudrate = 115200, timeout = 1)
time.sleep(1)

while True:
    try:
        ser.write(msg_enc)
        print("Mensaje enviado")
        time.sleep(10)
    except KeyboardInterrupt:
        break

ser.close()