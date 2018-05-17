from math import pi, cos, sin
import Koala


class Odometry:
    def __init__(self, koala):
        assert isinstance(koala, Koala.Koala), koala

        self.koala = koala
        self.left_pos = 0
        self.right_pos = 0
        self.angle = 0
        self.x = 0
        self.y = 0

        self.reset()

    def reset(self):
        self.koala.reset_counters()
        self.left_pos = int(self.koala.positions[1])
        self.right_pos = int(self.koala.positions[2])
        self.angle = pi / 2
        self.x = 0
        self.y = 0

    def step(self):
        _, left_pos, right_pos = self.koala.positions
        delta_pos_left = int(left_pos) - self.left_pos  # change in encoder value of left wheel
        delta_pos_right = int(right_pos) - self.right_pos  # change in encoder value of right wheel

        delta_left = delta_pos_left * self.koala.TICKS_LEFT
        delta_right = delta_pos_right * self.koala.TICKS_RIGHT
        delta_theta = (delta_right - delta_left) / self.koala.WHEEL_BASE
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
