import serial
import KoalaController
import signal
import sys


def signal_handler(signal, frame):
        #print('You pressed Ctrl+C!')
        koala.set_speed(0,0) 
        sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

ser = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_TWO,
        bytesize=serial.EIGHTBITS,
        xonxoff = True
)

mqtt_host = "192.168.1.186"

koala = KoalaController.KoalaController(mqtt_host, ser)
