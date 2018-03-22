import serial
import time
import curses
from math import pi


class Koala:
	def __init__(self, serial):
		self.rspeed = 0
		self.lspeed = 0
		
		self.width = 0.30  # m 0.28
		
		self.serial = serial
		self.set_speed(0, 0)
		
		self.full_circle_ticks = 21198

	def write(self, *arg):
		def response():
			out = ''
			time.sleep(0.1)
			while self.serial.inWaiting() > 0:
				out += self.serial.read(1)
			if out != '':
				print(out)
				return out.split(',')
				
		message = []
		for n, x in enumerate(arg):
			if n == 0:
				message.append(str(x))
			else:
				message.append(str(int(x)))
		
		self.serial.write(",".join(message) + '\r')
		return response()
	
	@property
	def speed(self):
		return self.write('E')
		
	def set_speed(self, l, r):
		l = self._clip(l)
		r = self._clip(r)
		self.write('D', l, r)
		
	def stop(self):
		return self.write('D', 0, 0)
	
	def rotate(self, angle):
		
			
		rate = angle / 360.0
		self.write('G', 0, 0)
		
		pos = self.write('H')
		finish = self.full_circle_ticks * rate * 0.92
		print("Finish = " + str(finish))
		while abs(int(self.write('H')[2])) < finish:
			time.sleep(0.001)
		self.write('D', 0, 0)
		return
		
	def follow_arch(self, radius):  # radius in m
		inner_radius = radius - self.width / 2
		outer_radius = radius + self.width / 2
		
		inner_circumference = self._circle_circumference(inner_radius)
		outer_circumference = self._circle_circumference(outer_radius)
		
		inner_pulses = self._to_pulses(inner_circumference)
		outer_pulses = self._to_pulses(outer_circumference)
		
		rate = inner_circumference / outer_circumference
		
		max_speed = 20
		acc = 255
		
		inner_max_speed = max_speed * rate
		inner_acc = acc * rate
		# print(locals())
		self.write('G', 0, 0)
		self.write('D', max_speed, max_speed * rate)
		#self.write('J', max_speed, acc, inner_max_speed, acc)
		#self.write('C', outer_pulses, inner_pulses)
		
		while True:
			pos = self.write('H')
			pos1 = int(pos[1])
			pos2 = int(pos[2])
			
			if (abs(pos1) or abs(pos2)) > outer_pulses:
				break
			time.sleep(0.001)
		self.write('D', 0, 0)
		
		
	@staticmethod
	def _to_pulses(length):
		return (length * 1000) / 0.045
	
	@staticmethod
	def _circle_circumference(radius):
		return 2 * radius * pi
		
	@staticmethod
	def _clip(value, min=-100, max=100):
		v = value
		if value > max:
			v = max
		if value < min:
			v = min
		return v




