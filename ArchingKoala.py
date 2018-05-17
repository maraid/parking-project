import Koala
from math import tan, atan, cos, acos, sin, pi
import numpy as np
import time
import threading


class ArchingKoala(Koala.Koala):
    MAX_SPEED = 0.3  # m/s
    MAX_SPEED_LOW = 0.12  # m/s
    MAX_ACCELERATION = 0.07  # m/s^2

    def __init__(self, serial, new_message_flag, stop_flag):
        Koala.Koala.__init__(self, serial)
        self.new_message_flag = new_message_flag
        self.stop_flag = stop_flag
        self.reach_pos_thread = None

        self.p = 0.0
        self.time_goal = 0.0
        self.delta_time = 0.0
        self.start_time = 0.0
        self.while_cycle_prev_time = 0.0
        self.program_start = time.time()
        self.while_cycle_time = 0.0

        self.obstacle = False
        self.set_speed(0, 0)

        self.circle_radius = 0.0
        self.circle_arch_length = 0.0
        self.circle_right = True
        self.circle_forward = True

    def reset(self):
        self.p = 0.0
        self.time_goal = 0.0
        self.delta_time = 0.0
        self.start_time = 0.0
        self.while_cycle_prev_time = 0.0
        self.program_start = time.time()
        self.while_cycle_time = 0.0
        self.obstacle = False
        self.set_speed(0, 0)
        self.circle_radius = 0.0
        self.circle_arch_length = 0.0
        self.circle_right = True
        self.circle_forward = True

    def callback(self, radius, length):
        self.reset()
        self.odometry.reset()
        self.set_parameters(radius, length)
        self.time_goal = self.circle_arch_length / self.MAX_SPEED_LOW

        if self.reach_pos_thread.is_alive:
            raise(RuntimeError, "reach_pos_thread hasn't ended in time")

        self.reach_pos_thread = threading.Thread(target=self.reach_pos)
        self.reach_pos_thread.start()

    def set_parameters(self, radius, length):
        if abs(radius) == 0.0:
            radius = 0.0000001

        if abs(length) == 0.0:
            length = 0.0000001

        if radius > 0:
            self.circle_right = True
        else:
            self.circle_right = False

        if length > 0:
            self.circle_forward = True
        else:
            self.circle_forward = False

        self.circle_radius = abs(radius)
        self.circle_arch_length = abs(length)
        print("rad: " + str(self.circle_radius) + " length: " + str(self.circle_arch_length))

    def reach_pos(self, new_message_flag, stop_flag):
        while (self.delta_time <= self.time_goal) and not self.obstacle:
            if new_message_flag.is_set():
                break
            self.detect_obstacle()
            self.odometry.step()
            self.while_cycle_prev_time = time.time()
            self.delta_time = time.time() - self.program_start
            self.calc_p()
            next_x, next_y = self.arch(self.p)
            self.get_small_radius(next_x, next_y)

            self.while_cycle_time = time.time() - self.while_cycle_prev_time

        self.set_speed(0, 0)
        stop_flag.set()
        return

    def calc_p(self):
        if self.time_goal == 0:
            self.time_goal = 0.000001
        print("time_goal: " + str(self.time_goal) + " time_start: " + str(self.start_time))
        if self.delta_time == 0:
            self.p = 0
        else:
            self.p = (self.delta_time + self.while_cycle_time) / (self.time_goal - self.start_time)

    def arch(self, p):
        # max speed 0.3 de mi csak mondjuk konstans 0.1-el megyunk -> kiszamolni hogy mennyi ideig tart elerni a celt
        circle_circumference = self.circle_radius * 2 * pi
        max_rad = 2 * pi * (self.circle_arch_length / circle_circumference)

        if self.circle_forward:
            if self.circle_right:
                y = self.circle_radius * cos((max_rad * p) - (pi / 2))
                x = self.circle_radius * sin((max_rad * p) - (pi / 2)) + self.circle_radius  # see geogebra example
            else:
                y = self.circle_radius * cos((max_rad * (1.0 - p)) - (pi / 2))
                x = self.circle_radius * sin(
                    (max_rad * (1.0 - p)) - (pi / 2)) - self.circle_radius  # see geogebra example
        else:
            if self.circle_right:
                y = -self.circle_radius * cos((max_rad * (1 - p)) - (pi / 2))
                x = -self.circle_radius * sin(
                    (max_rad * (1 - p)) - (pi / 2)) + self.circle_radius  # see geogebra example
            else:
                y = -self.circle_radius * cos((max_rad * p) - (pi / 2))
                x = -self.circle_radius * sin((max_rad * p) - (pi / 2)) - self.circle_radius  # see geogebra example

        return x, y

    def get_small_radius(self, x, y):
        # koordinatak eltolasa a jelenlegi pozicioba -> konnyebb szamolas
        start_x = 0.0
        start_y = 0.0

        end_x = float(x - self.odometry.x)
        end_y = float(y - self.odometry.y)

        x_a = float(-1 * tan(self.odometry.angle - (pi / 2.0)))
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
        eqx = np.linalg.solve(a, b)

        center_x = eqx[0]
        center_y = eqx[1]
        center_radius = (center_x ** 2 + center_y ** 2) ** 0.5
        self.odometry.step()

        start_end_euclid_distance = ((x - self.odometry.x) ** 2 + (y - self.odometry.y) ** 2) ** 0.5

        circle_angle = acos((center_radius ** 2 + center_radius ** 2 - start_end_euclid_distance ** 2) / (
                2 * center_radius * center_radius))

        if self.circle_right:
            left_radius = center_radius + (self.WHEEL_BASE / 2.0)
            right_radius = center_radius - (self.WHEEL_BASE / 2.0)
        else:
            left_radius = center_radius - (self.WHEEL_BASE / 2.0)
            right_radius = center_radius + (self.WHEEL_BASE / 2.0)

        left_circumference = 2 * left_radius * pi
        right_circumference = 2 * right_radius * pi

        left_arch_length = left_circumference * (circle_angle / (2 * pi))
        right_arch_length = right_circumference * (circle_angle / (2 * pi))

        left_speed = left_arch_length / (self.while_cycle_time * 3)
        right_speed = right_arch_length / (self.while_cycle_time * 3)

        left_wheel_speed = left_speed / 0.0045
        right_wheel_speed = right_speed / 0.0045

        if self.circle_forward:
            self.set_speed(left_wheel_speed, right_wheel_speed)
        else:
            self.set_speed(-1.0 * left_wheel_speed, -1.0 * right_wheel_speed)

    def detect_obstacle(self):
        sensor_data = self.write('N')
        print('sensor_data: ' + str(sensor_data))
        sensor_sum = 0
        for i in range(0, len(sensor_data)):
            if i > 0:
                sensor_sum = sensor_sum + int(sensor_data[i])
        sensor_avg = sensor_sum / 16.0

        if sensor_avg > 50:
            self.obstacle = True
            self.set_speed(0, 0)
        else:
            self.obstacle = False
