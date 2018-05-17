import time
import Odometry
from math import pi


class Koala:
    WHEEL_RADIUS_LEFT = 4.19 / 100  # m
    WHEEL_RADIUS_RIGHT = 4.19 / 100  # m
    WHEEL_BASE = 30.6 / 100  # m

    TICKS_PER_REVOLUTION = 5850.0

    TICKS_LEFT = 2 * pi * WHEEL_RADIUS_LEFT / TICKS_PER_REVOLUTION  # position change/tick
    TICKS_RIGHT = 2 * pi * WHEEL_RADIUS_RIGHT / TICKS_PER_REVOLUTION

    def __init__(self, serial):
        self.serial = serial
        self.odometry = Odometry.Odometry(self)

    def write(self, *arg):
        def response():
            out = ''
            time.sleep(0.1)
            while self.serial.inWaiting() > 0:
                out += self.serial.read(1)
            if out != '':
                out = out.split("\r\n")[0]
                out = out.split(',')
                out = out[:1] + [int(o) for o in out[1:]]
                return out

        message = []
        for n, x in enumerate(arg):
            if n == 0:
                message.append(str(x))
            else:
                message.append(str(int(x)))
        self.serial.write((",".join(message) + '\r').encode())
        return response()

    @property
    def speed(self):
        return self.write('E')

    def set_speed(self, left, right):
        return self.write('D', self.clip(left), self.clip(right))

    def stop(self):
        return self.write('D', 0, 0)

    def reset_counters(self, left=0, right=0):
        assert isinstance(left, int), left
        assert isinstance(right, int), right
        return self.write('G', left, right)

    @staticmethod
    def clip(value, min=-100, max=100):
        if value > max:
            return max
        elif value < min:
            return min
        return value

    @property
    def positions(self):
        return_val = self.write('H')
        if return_val is None:
            return_val = ['x', '0', '0']
        if len(return_val) < 3:
            return_val = ['x', '0', '0']
        print("ret_val length: " + str(len(return_val)))
        print("return: " + str(return_val))
        return return_val
