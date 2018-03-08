import serial
import time
import curses
from math import pi


class Koala:
	def __init__(self, serial):
		self.rspeed = 0
		self.lspeed = 0
		
		self.width = 0.28  # m
		
		self.serial = serial
		self.set_speed(0, 0)

	def write(self, *arg):
		def response():
			out = ''
			time.sleep(0.1)
			while self.serial.inWaiting() > 0:
				out += self.serial.read(1)
			if out != '':
				return out
		self.serial.write(",".join([str(x) if n == 0 else str(int(x)) for n, x in enumerate(arg)]) + '\r')
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
	
	def rotate_in_place(self, angle):
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
		acc = 64
		
		inner_max_speed = max_speed * rate
		inner_acc = acc * rate
		# print(locals())
		self.write('G', 0, 0)
		self.write('J', max_speed, acc, inner_max_speed, inner_acc)
		self.write('C', outer_pulses, inner_pulses)
		
		
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




