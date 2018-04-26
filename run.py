import serial
import time
import Koala
import threading
import matplot

ser = serial.Serial(
		port='/dev/ttyUSB0',
		baudrate=9600,
		parity=serial.PARITY_NONE,
		stopbits=serial.STOPBITS_TWO,
		bytesize=serial.EIGHTBITS,
		xonxoff = True
)


koala = Koala.Koala(ser)
koala.set_speed(20, 20)
while True:
	koala.odo_step()
	print("res_X: " + str(koala.x) + "\nres_Y: " + str(koala.y))
	if koala.y > 1:
		koala.stop()
		break




# koala.follow_arch(0.3)

try:
	main()
except KeyboardInterrupt:
	koala.stop()
	stop_flag.set()

"""
input=1

while 1 :
    # get keyboard input
    input = raw_input(">> ")
        # Python 3 users
        # input = input(">> ")
    if input == 'exit':
        ser.close()
        exit()
    else:
		
		
		ser.write(input + '\r')
		# send the character to the device
		# (note that I happend a \r\n carriage return and line feed to the characters - this is requested by my device)
		
		out = ''
		# let's wait one second before reading output (let's give device time to answer)
		time.sleep(1)
		while ser.inWaiting() > 0:
			out += ser.read(1)
		if out != '':
			print ">>" + out

#	screen = curses.initscr()
#	curses.cbreak()
#	screen.nodelay(True)
#	curses.noecho()
#	screen.keypad(True)
"""