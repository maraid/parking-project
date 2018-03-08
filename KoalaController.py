import curses

class KoalaController:
	def __init__(self):
		curses.noecho()
		curses.cbreak()
		
		self.screen = curses.initscr()
		self.screen.nodelay(True)
		self.screen.keypad(True)
	
	def start(self, koala):
		while True:
			char = self.screen.getch()
			if char == curses.KEY_IC:
				koala.inc
			elif char == curses.KEY_DC:
				koala.lspeed -= 1
			elif char == curses.KEY_PPAGE:
				koala.rspeed += 1
			elif char == curses.KEY_NPAGE:
				koala.rspeed -= 1
			elif char == curses.KEY_UP:
				koala.rspeed += 1
				koala.lspeed += 1
			elif char == curses.KEY_DOWN:
				rspeed -= 1
				lspeed -= 1
			elif char == curses.KEY_BACKSPACE:
				lspeed = rspeed = 0
			w(lspeed, rspeed)
		
		
	