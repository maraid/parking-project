import serial
import time
from math import pi, cos, sin, tan, atan
import numpy as np


class Koala:
    def __init__(self, serial):
        self.rspeed = 0
        self.lspeed = 0

        self.serial = serial

        self.set_speed(0, 0)

        self.full_circle_ticks = 21198

        # ODOMETRY
        self.max_speed = 0.3  # m/s
        self.max_acceleration = 0.07  # m/s^2

        self.wheel_radius_left = 4.19 / 100  # m
        self.wheel_radius_right = 4.19 / 100  # m
        self.wheel_base = 30.6 / 100  # m

        self.tick_per_revolution = 5850.0

        self.tick_size_left = 2 * pi * self.wheel_radius_left / self.tick_per_revolution  # position change/tick
        self.tick_size_right = 2 * pi * self.wheel_radius_right / self.tick_per_revolution

        self.speed_factor = 0.01 / ((self.tick_size_left + self.tick_size_right) / 2)  # change from m/s to tick/second

        self.write('G', 0, 0)
        self.left_pos = int(self.poses[1])
        self.right_pos = int(self.poses[2])
        self.angle = pi/2
        self.x = 0
        self.y = 0

        self.circle_radius = 0.0
        self.circle_arch_length = 0.0

    def odo_reset(self, x, y, theta):
        self.write('G', 0, 0)
        self.left_pos = int(self.poses[1])
        self.right_pos = int(self.poses[2])
        self.angle = theta
        self.x = x
        self.y = y

    def this_is_mqtt_visual_topic_callback(self):
        # mqtt get info from visual team topic
        raise NotImplementedError()

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

    def set_speed(self, left, right):
        self.write('D', self.clip(left), self.clip(right))

    def stop(self):
        return self.write('D', 0, 0)

    def arch(self, p, t=3):
        circle_circumference = self.circle_radius*2*pi
        max_rad = 2 * pi * (self.circle_arch_length / circle_circumference)

        x = self.circle_radius * cos(max_rad * p)
        y = self.circle_radius * sin(max_rad * p)
        return x, y

    def get_small_radius(self, x, y):
        # 1 egyenes egyenlete
        # tan(self.angle) x = 0

        # 2 egyenes egyenlete (1-re meroleges)
        #            -1*x_a            r_a y_a
        # tan( self.angle - (pi/2) ) x + 0 = y

        # 3 egyenes (jelenlegi pont, kovetkezo celpont kozotti egyenes)
        # ( end_y - start_y ) / ( end_x - start_x ) x = 0

        # 3as felezopontja:  mid_x = start_x + ((end_x - start_x) / 2)
        #                   mid_y = start_y + ((end_y - start_y) / 2)

        # 4 egyenes (3as felezo merolegese)
        # meredekseg = tan(atan( 3as meredeksege) - (pi/2)) x
        # eltolas = mid_y - meredekseg*mid_x
        #  -1*x_b        r_b     y_b
        # meredekseg x + eltolas = y

        # Solve the system of equations 3 * x0 + x1 = 9 and x0 + 2 * x1 = 8:
        # a = np.array([[3,1], [1,2]])
        # b = np.array([9,8])
        # x = np.linalg.solve(a, b)

        # kor kozeppontja(center): 2es 4es metszespontja (numpy megoldja)
        # kor sugara: sqrt(center_x^2 + center_y^2)

        # koordinatak eltolasa a jelenlegi pozicioba -> konnyebb szamolas
        start_x = 0.0
        start_y = 0.0

        end_x = float(x - self.x)
        end_y = float(y - self.y)

        x_a = float(-1 * tan(self.angle - (pi / 2.0)))
        y_a = 1.0
        r_a = 0.0

        mid_x = end_x / 2.0
        mid_y = end_y / 2.0

        b_steepness = tan(atan((end_y - start_y) / (end_x - start_x)) - (pi / 2.0))
        b_offset = mid_y - (b_steepness * mid_x)
        x_b = -1.0 * b_steepness
        y_b = 1.0
        r_b = b_offset

        a = np.array([[x_a, y_a],
                      [x_b, y_b]])
        b = np.array([r_a, r_b])
        x = np.linalg.solve(a, b)

        center_x = x[0]
        center_y = x[1]
        radius = (center_x ** 2 + center_y ** 2) ** 0.5

        return radius

    @staticmethod
    def clip(value, min=-100, max=100):
        if value > max:
            return max
        elif value < min:
            return min
        return value

    def odo_step(self):
        _, left_pos, right_pos = self.poses
        delta_pos_left = int(left_pos) - self.left_pos  # change in encoder value of left wheel
        delta_pos_right = int(right_pos) - self.right_pos  # change in encoder value of right wheel

        delta_left = delta_pos_left * self.tick_size_left
        delta_right = delta_pos_right * self.tick_size_right
        delta_theta = (delta_right - delta_left) / self.wheel_base
        theta2 = self.angle + delta_theta * 0.5
        delta_x = (delta_left + delta_right) * 0.5 * cos(theta2)
        delta_y = (delta_left + delta_right) * 0.5 * sin(theta2)

        self.x = self.x + delta_x
        self.y = self.y + delta_y

        self.angle = self.angle + delta_theta

        if self.angle > pi:
            self.angle = self.angle - 2 * pi
        if self.angle <= -pi:
            self.angle = self.angle + 2 * pi

        self.left_pos = int(left_pos)
        self.right_pos = int(right_pos)

    @property
    def poses(self):
        return self.write('H')
