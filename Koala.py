import serial
import time
from math import pi, cos, sin, tan, atan
import numpy as np
import time


class Koala:
    def __init__(self, serial):
        self.rspeed = 0
        self.lspeed = 0
        
        self.p = 0.0
        self.time_goal = 0.0
        self.time_act = 0.0
        self.time_start = 0.0
        self.while_cycle_prev_time = 0.0
        self.program_start = time.time()
        self.while_cycle_time = 0.0001 #ki kell merni hogy a valosagban mennyi es beirni

        self.serial = serial

        self.set_speed(0, 0)

        self.full_circle_ticks = 21198

        # ODOMETRY
        self.max_speed = 0.3  # m/s
        self.max_speed_low = 0.1 #m/s
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
        self.p = 0.0

    def this_is_mqtt_visual_topic_callback(self):
        # mqtt get info from visual team topic
        #raise NotImplementedError()
        self.circle_radius = 1.0
        self.circle_arch_length = 3.14
        self.program_start = time.time()
        self.time_goal =   self.circle_arch_length / self.max_speed_low
        
    def calc_p(self):
        if(self.time_act == 0):
            self.p = 0
        else:
            self.p = (self.time_goal - self.time_start) / (self.time_act + self.while_cycle_time)

    def write(self, *arg):
        def response():
            out = ''
            time.sleep(0.1)
            while self.serial.inWaiting() > 0:
                out += self.serial.read(1)
            if out != '':
                #print(out)
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

    def arch(self, p):
        #max speed 0.3 de mi csak mondjuk konstans 0.1-el megyunk -> kiszamolni hogy mennyi ideig tart elerni a celt
        circle_circumference = self.circle_radius*2*pi
        max_rad = 2 * pi * (self.circle_arch_length / circle_circumference)

        x = self.circle_radius * cos((max_rad * p) - ( pi / 2 )) 
        y = self.circle_radius * sin((max_rad * p) - ( pi / 2 )) + 1 #see geogebra example
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
        #print("self: (" + str(self.x) + "," + str(self.y) + ")")
        #print("end: (" + str(end_x) + "," + str(end_y) + ")")
        
        
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
        center_radius = (center_x ** 2 + center_y ** 2) ** 0.5
        if self.circle_radius > 0:
            left_radius = center_radius + (self.wheel_base/2.0) 
            right_radius = center_radius - (self.wheel_base/2.0) 
        else:
            left_radius = center_radius - (self.wheel_base/2.0) 
            right_radius = center_radius + (self.wheel_base/2.0) 
            
        center_circumference = 2*center_radius*pi
        center_full_time = center_circumference / self.max_speed_low     
        center_distance = center_circumference * (self.while_cycle_time / center_full_time)
        
        left_circumference = 2*left_radius*pi
        left_full_time = left_circumference / self.max_speed_low     
        left_distance = left_circumference * (self.while_cycle_time / left_full_time)
        
        right_circumference = 2*right_radius*pi
        right_full_time = right_circumference / self.max_speed_low     
        right_distance = right_circumference * (self.while_cycle_time / right_full_time)
        
        left_wheel_speed = (left_distance/self.while_cycle_time)/0.0045
        right_wheel_speed = (right_distance/self.while_cycle_time)/0.0045
        
        self.set_speed(left_wheel_speed, right_wheel_speed) 

    @staticmethod
    def clip(value, min=-100, max=100):
        if value > max:
            return max
        elif value < min:
            return min
        return value
        
    def reach_pos(self):
        print(self.time_goal)
        while(self.time_act < self.time_goal):
            self.odo_step()
            self.while_cycle_prev_time = time.time()
            self.time_act = time.time() - self.program_start
            self.calc_p()
            next_x, next_y = self.arch(self.p)
            self.get_small_radius(next_x, next_y)
            
            self.while_cycle_time = time.time() - self.while_cycle_prev_time
            
#   def step(self):
#       self.odo_step()
#       self.p = self.p + 0.25
#       next_x, next_y = self.arch(self.p)
#       radius = self.get_small_radius(next_x, next_y)
#       print("small radius: " + str(radius))
#       omega = 10.0/radius
#       
#       left_speed = omega*(radius - (self.wheel_base/2.0))
#       right_speed = omega*(radius + (self.wheel_base/2.0))
#       
#       self.set_speed(left_speed,right_speed)
#       while abs(self.x - next_x) > 0.1 and abs(self.y - next_y) > 0.1:
#           print("pos_error: (" + str(abs(self.x - next_x)) + "," + str(abs(self.y - next_y)) + ")")
#           self.odo_step()
 
           
    def step(self):
        self.odo_step()
        self.p = self.p + 0.2
        print("p: " + str(self.p))
        
        radius = self.circle_radius
        omega = 10.0/radius 
        left_speed = omega*(radius - (self.wheel_base/2.0))
        right_speed = omega*(radius + (self.wheel_base/2.0))
        self.set_speed(left_speed,right_speed)
        
        #checkx, checky = self.arch(self.p) 
        #error = ((self.x - checkx)  ** 2 + (self.y - checky) ** 2) ** 0.5
        while((((self.x - self.arch(self.p)[0])  ** 2 + (self.y - self.arch(self.p)[1]) ** 2) ** 0.5) > 0.2) :
            error = (((self.x - self.arch(self.p)[0])  ** 2 + (self.y - self.arch(self.p)[1]) ** 2) ** 0.5)
            self.odo_step()
            next_x, next_y = self.arch(self.p)
            radius = self.get_small_radius(next_x, next_y)
            print("small radius: " + str(radius) + "p: " + str(self.p) + " error: " + str(error))
            omega = 10.0/radius
            
            left_speed = omega*(radius - (self.wheel_base/2.0))
            right_speed = omega*(radius + (self.wheel_base/2.0))
            
            self.set_speed(left_speed,right_speed)
            
            
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
