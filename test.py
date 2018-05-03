import Koala
import serial
import time
from math import pi

ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.EIGHTBITS,
    xonxoff = True
)

start = time.time()
while True:
    curr = time.time()
    elapsed = curr - start
    print(elapsed)
    time.sleep(0.1)
    if elapsed > 3:
        break

#koala = Koala.Koala(ser)
#
#koala.x = 0
#koala.y = 0
#koala.angle = pi/2 + pi/4
#print(koala.get_small_radius(5, 2))
#
#print("arch testing: ")
#koala.this_is_mqtt_visual_topic_callback()
#print(koala.arch(0))
#print(koala.arch(0.25))
#print(koala.arch(0.5))
#print(koala.arch(0.75))
#print(koala.arch(1)) 