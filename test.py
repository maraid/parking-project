import Koala
import serial
from math import pi

ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.EIGHTBITS,
    xonxoff = True
)


koala = Koala.Koala(ser)

koala.x = 0
koala.y = 0
koala.angle = pi/2 + pi/4
print(koala.get_small_radius(5, 2))
