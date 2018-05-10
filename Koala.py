import serial
import time
from math import pi, cos, sin, tan, atan
import numpy as np
import time
import MQTTClient
import threading


class Koala:
    def __init__(self, serial):
		self.mqtt = MQTTClient.MQTTClient("HOST IP")
		self.mqtt_thread = threading.Thread(target=self.mqtt.loop_forever)
		self.mqtt_thread.start()
		self.new_unhandled_message = threading.Event()
		
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
        self.max_speed_low = 0.12 #m/s
        self.speed_dependent_time_correction = 1 # (speed: 0.25m/s, radius: 0.5): 1.05; (speed: 0.12m/s, radius: 0.5): 1.068
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
        self.circle_right = True
        self.circle_forward = True
	
	def mqtt_command(self, client, userdata, msg):
		self.new_unhandled_message.set()
		print(msg.topic+" "+str(msg.payload))
		paylaod = json.loads(msg.payload)
		r = float(paylaod.get("R"))
		l = float(payload.get("L"))
		self.this_is_mqtt_visual_topic_callback(r, l)
	
    def odo_reset(self, x, y, theta):
        self.write('G', 0, 0)
        self.left_pos = int(self.poses[1])
        self.right_pos = int(self.poses[2])
        self.angle = theta
        self.x = x
        self.y = y
        self.p = 0.0

    def this_is_mqtt_visual_topic_callback(self, radius, length):
        # mqtt get info from visual team topic
        #raise NotImplementedError()
        # radius = -0.5
        # length = +2.0 * abs(radius) * pi / 2.0
		self.new_unhandled_message.clear()
		
        if(radius > 0):
            self.circle_right = True
        else:
            self.circle_right = False
            
        if(length > 0):
            self.circle_forward = True
        else:
            self.circle_forward = False
        
        self.circle_radius = abs(radius)
        self.circle_arch_length = abs(length)
        
        print("rad: " + str(self.circle_radius) + " length: " + str(self.circle_arch_length))
        
        self.program_start = time.time()
        self.time_goal =   self.circle_arch_length / self.max_speed_low
        
    def calc_p(self):
        if(self.time_act == 0):
            self.p = 0
        else:
            self.p =  (self.time_act + self.while_cycle_time) / (self.time_goal - self.time_start)

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

        
        if(self.circle_forward == True):
        
            if(self.circle_right == True):
                y = self.circle_radius * cos((max_rad * p) - ( pi / 2 )) 
                x = self.circle_radius * sin((max_rad * p) - ( pi / 2 )) + self.circle_radius #see geogebra example
            else:
                y = self.circle_radius * cos((max_rad * (1.0-p)) - ( pi / 2 )) 
                x = self.circle_radius * sin((max_rad * (1.0-p)) - ( pi / 2 )) - self.circle_radius #see geogebra example
        
        else:
        
            if(self.circle_right == True):
                y = -self.circle_radius * cos((max_rad * (1-p)) - ( pi / 2 )) 
                x = -self.circle_radius * sin((max_rad * (1-p)) - ( pi / 2 )) + self.circle_radius #see geogebra example
            else:
                y = -self.circle_radius * cos((max_rad * p) - ( pi / 2 )) 
                x = -self.circle_radius * sin((max_rad * p) - ( pi / 2 )) - self.circle_radius #see geogebra example

        
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
        
        if (self.circle_right == True):
            left_radius = center_radius + (self.wheel_base/2.0) 
            right_radius = center_radius - (self.wheel_base/2.0) 
        else:
            left_radius = center_radius - (self.wheel_base/2.0) 
            right_radius = center_radius + (self.wheel_base/2.0) 
            
        #print("cr: " + str(center_radius) + " lr: " + str(left_radius) + " rr: " + str(right_radius));
            
        center_circumference = 2*center_radius*pi
        center_full_time = center_circumference / self.max_speed_low     

        
        left_circumference = 2*left_radius*pi
        left_speed = (left_circumference / center_circumference) * self.max_speed_low
        

        
        right_circumference = 2*right_radius*pi
        right_speed = (right_circumference / center_circumference ) * self.max_speed_low

        
        #print("ccir: " + str(center_circumference) + " lcir: " + str(left_circumference) + " rcir: " + str(right_circumference));
        
        left_wheel_speed = left_speed/0.0045
        right_wheel_speed = right_speed/0.0045
        
        #print("lws: " + str(left_wheel_speed) + " rws: " + str(right_wheel_speed))
        #print(str(self.while_cycle_time))
        #print(" ")
        
        if(self.circle_forward == True):
            self.set_speed(left_wheel_speed, right_wheel_speed) 
        else:
            self.set_speed(-1.0*left_wheel_speed, -1.0*right_wheel_speed) 
        
        

    @staticmethod
    def clip(value, min=-100, max=100):
        if value > max:
            return max
        elif value < min:
            return min
        return value
        
    def reach_pos(self):
        print(self.time_goal)
        while(self.time_act < (self.time_goal*self.speed_dependent_time_correction)):
			if self.new_unhandled_message.is_set():
				break
			
            #print("p: " + str(self.p) + " time_act: " + str(self.time_act) + " wct: " + str(self.while_cycle_time))
            self.odo_step()
            self.while_cycle_prev_time = time.time()
            self.time_act = time.time() - self.program_start
            self.calc_p()
            next_x, next_y = self.arch(self.p)
            self.get_small_radius(next_x, next_y)
            error = ((next_x-self.x)**2 + ((next_y-self.y)**2))**0.5
            #print("self: " + str(self.x) + "," + str(self.y) + " dest: " + str(next_x) + "," + str(next_y) + "error: " + str(error))
            print(str(next_x) + "," + str(next_y) + "," + str(self.x) + ", " + str(self.y) + ", " + str(self.p) + ", " + str(self.while_cycle_time)+ ", " + str(self.lspeed)+ ", " + str(self.rspeed))
            
            self.while_cycle_time = time.time() - self.while_cycle_prev_time
            
        self.set_speed(0,0)
        return
 
           
            
            
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