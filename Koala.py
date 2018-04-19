import serial
import time
import curses
from math import pi, cos, sin


class Koala:
    def __init__(self, serial):
        self.rspeed = 0
        self.lspeed = 0

        self.serial = serial

        self.set_speed(0, 0)

        self.full_circle_ticks = 21198

        # ODOMETRY
        self.write('G', 0, 0)
        self.max_speed = 0.3  # m/s
        self.max_acceleration = 0.07  # m/s^2

        self.wheel_radius_left = 4.19 / 100  # m
        self.wheel_radius_right = 4.19 / 100  # m
        self.wheel_base = 30.6 / 100  # m

        self.tick_per_revolution = 5850.0

        self.tick_size_left = 2 * pi * self.wheel_radius_left / self.tick_per_revolution  # position change/tick
        self.tick_size_right = 2 * pi * self.wheel_radius_right / self.tick_per_revolution

        self.speed_factor = 0.01 / ((self.tick_size_left + self.tick_size_right) / 2)  # change from m/s to tick/second

        self.left_pos = int(self.poses[1])
        self.right_pos = int(self.poses[2])
        self.angle = pi / 2
        self.res_X = 0.0
        self.res_Y = 0.0

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
        inner_radius = radius - self.wheelBase / 2
        outer_radius = radius + self.wheelBase / 2

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
        # self.write('J', max_speed, acc, inner_max_speed, acc)
        # self.write('C', outer_pulses, inner_pulses)

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

    def step(self):
        _, left_pos, right_pos = self.poses
        delta_pos_left = int(left_pos) - self.left_pos  # change in encoder value of left wheel
        delta_pos_right = int(right_pos) - self.right_pos  # change in encoder value of right wheel

        delta_left = delta_pos_left * self.tick_size_left
        delta_right = delta_pos_right * self.tick_size_right
        delta_theta = (delta_right - delta_left) / self.wheel_base
        theta2 = self.angle + delta_theta * 0.5
        delta_x = (delta_left + delta_right) * 0.5 * cos(theta2)
        delta_y = (delta_left + delta_right) * 0.5 * sin(theta2)

        self.res_X = self.res_X + delta_x
        self.res_Y = self.res_Y + delta_y

        self.angle = self.angle + delta_theta

        if self.angle > pi:
            self.angle = self.angle - 2 * pi
        if self.angle <= -pi:
            self.angle = self.angle + 2 * pi

        self.left_pos = int(left_pos)
        self.right_pos = int(right_pos)

    # def read_steps(self, stop_flag, lock):
    #     while True:
    #         if stop_flag.is_set():
    #             print("fasz")
    #             return
    #         lock.acquire()
    #         self.step()
    #         print("res_X: " + str(self.res_X) + "\nres_Y: " + str(self.res_Y))
    #         lock.release()

    @property
    def poses(self):
        poses = self.write('H')
        return poses
