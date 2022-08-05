import odrive
from odrive.enums import *
import time
import socket
import atexit
import math
import threading
from queue import Queue
import matplotlib.pyplot as plt


class Odrive:
    def __init__(self) -> None:
        self.drive = odrive.find_any()
        time.sleep(1)
        self.init_odrive()
        self.set_zero()
        self.goto_Home()
        self.set_zero()
        self.set_closed_loop()

    def init_odrive(self) -> None:
        self.drive.axis0.controller.config.input_mode = 1
        self.drive.axis1.controller.config.input_mode = 1
        self.drive.axis0.motor.config.current_lim = 70  # 70
        self.drive.axis1.motor.config.current_lim = 70  # 70
        self.drive.axis0.motor.config.current_lim_margin = 30
        self.drive.axis1.motor.config.current_lim_margin = 30
        self.drive.axis0.controller.config.vel_limit = 10 # 10
        # self.drive.axis0.controller.config.vel_limit_tolerance = 0  # old value 1.2000000476837158, not running save_configuration() so should revert to this value if this line isn't run
        self.drive.axis0.controller.config.enable_overspeed_error = False
        self.drive.axis1.controller.config.enable_overspeed_error = False
        self.drive.axis1.controller.config.vel_limit = 10 # 10
        self.drive.axis0.controller.config.pos_gain = 20  # 30
        self.drive.axis1.controller.config.pos_gain = 20  # 30

    def set_zero(self) -> None:
        self.drive.axis0.encoder.set_linear_count(0)
        self.drive.axis1.encoder.set_linear_count(0)
        print(self.drive.axis0.encoder.shadow_count)
        print(self.drive.axis1.encoder.shadow_count)

    def set_idle(self) -> None:
        self.drive.axis0.requested_state = AXIS_STATE_IDLE
        self.drive.axis1.requested_state = AXIS_STATE_IDLE

    def set_closed_loop(self) -> None:
        self.drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(1)

    def goto_raw(self, mot_0, mot_1) -> None:
        # print(time.time())
        self.drive.axis0.controller.input_pos = mot_0
        # print(time.time())
        self.drive.axis1.controller.input_pos = mot_1
        # print(time.time())

    def goto_Home(self) -> None:
        # self.drive.axis0.controller.config.vel_limit = 2
        self.drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.drive.axis1.requested_state = AXIS_STATE_IDLE
        
        print(f"Starting Up")

        time.sleep(1)

        prevShadowCount = self.drive.axis1.encoder.shadow_count + 4000
        currShadowCount = self.drive.axis1.encoder.shadow_count
        increment = 0.3
        while currShadowCount != prevShadowCount:
            i = self.drive.axis0.encoder.shadow_count/4000 - increment
            self.drive.axis0.controller.input_pos = i
            prevShadowCount = currShadowCount
            time.sleep(0.2)
            currShadowCount = self.drive.axis1.encoder.shadow_count
        # self.drive.axis0.controller.config.vel_limit = 10
        time.sleep(0.5)
        self.drive.axis0.requested_state = AXIS_STATE_IDLE
        self.drive.axis1.requested_state = AXIS_STATE_IDLE
        time.sleep(0.5)
        print(f"Startup complete")

class Server:
    def __init__(self) -> None:
        self.gain_changed = False
        self.armed = False
        self.armed = True
        self.x_offset = 0
        self.x_multiplier = -98/1 # -98/10  # NOTE: Remove the /10  
        self.y_offset = 0
        self.y_multiplier = 85/1 # 85/10  # NOTE: Remove the /10  
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.udp_host = socket.gethostname()
        self.udp_port = 59200
        self.sock.bind((self.udp_host, self.udp_port))
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 0)   #   set buffer size to length of message. only receive one message in queue
        self.init_odrive()
        atexit.register(self.at_exit)
        self.loopPoints = [0.5,1,1,0.5,0.5,0,0,0.5]
        self.iteration = 0
        self.speeds = []
        self.times  =[]
        self.q_angles = Queue()
        oDriveControlThread = threading.Thread(target = self.send_to_odrive, args = (self.q_angles,))
        oDriveControlThread.start()
        oDriveMotorSpeedThread = threading.Thread(target = self.calculateStrikerVelocity, args = ())
        oDriveMotorSpeedThread.start()
        self.control_loop()

    def init_odrive(self) -> None:
        self.drive = Odrive()

    def control_loop(self) -> None:
        timer = time.time()
        # hasSent = False
        # hasSent2=False
        # time.sleep(1)
        # target = [0.2, 0.7]
        # angle_0, angle_1 = self.calculate_angles(target[0], target[1])
        # if self.armed:
        #     self.drive.goto_raw(angle_0, angle_1)
        # time.sleep(5)
        while True:
            # print(self.calculateStrikerVelocity(self.drive.drive.axis0.encoder.vel_estimate, self.drive.drive.axis1.encoder.vel_estimate), time.time()-timer)
            # print("cpp fps is "+ str(1/(time.time()-timer)))
            timer= time.time()
            try:
                
                #normal
                data_encoded, _ = self.sock.recvfrom(1024)
                data_dirty = data_encoded.decode()
                data_clean = data_dirty.strip('(').strip(')').split(',')
                target = [0, 0]
                target[0] = float(data_clean[0])
                target[1] = float(data_clean[1])

                if target[0] > 1 or target[0] < 0 or target[1] > 1 or target[1] < 0:
                    print(f"{target} outside 0 to 1 range!")
                    exit(2)
                if math.isnan(target[0]) or math.isnan(target[1]):
                    print(f"{target} contains NaN!")
                    exit(2)
                #print(f"Received Message: {data_clean}, type: {type(data_clean)}")
                print(f"Parsed Message: {target}, type: {type(target)}")

                # data_clean = [0.0, 0.0, 0.2, 0.7]
                # target = [0.02, 0.1]
                
                self.q_angles.queue.clear()
                if len(data_clean) == 4:
                    striker = [0, 0]
                    striker[0] = float(data_clean[2])
                    striker[1] = float(data_clean[3])

                    x = target[0] - striker[0]
                    y = target[1] - striker[1]
                    x_angle = math.atan2(x, y)
                    y_angle = math.atan2(y, x)
                    
                    stepsize = 0.2
                    if x_angle == 0:
                        total_stepsize_count = math.floor(abs(y/stepsize))
                    
                    else:
                        total_stepsize_count = math.floor(abs((target[0]-striker[0])/math.sin(x_angle) / stepsize))
                    # total_stepsize_count = 0
                    # print(f"total stepsize count: {total_stepsize_count}")
                    # print(math.sin(x_angle) * stepsize)
                    for i in range(total_stepsize_count):
                        x_new = striker[0] + math.sin(x_angle) * stepsize*(i+1)
                        # x_new = target[0]
                        # if i % 2 == 0:
                        #     y_new = striker[1] + math.sin(y_angle) * stepsize *(i+1)
                        # else:
                        #     y_new = striker[1] + math.sin(y_angle) * stepsize *(i)
                        # y_new = striker[1] + math.sin(y_angle) * stepsize *(i+1-i%5)
                        y_new = striker[1] + math.sin(y_angle) * stepsize *(i+1)
                        self.q_angles.put([x_new, y_new])
                    self.q_angles.put([target[0],target[1]])
                    # print("hello")
                    # time.sleep(100)
                else:
                    self.q_angles.put([target[0], target[1]])
                    # time.sleep(100)
                # self.q_angles.put([target[0], target[1]])
                
                # self.drive.drive.axis0.controller.input_pos = self.drive.drive.axis0.encoder.shadow_count + 3
                # timer_test = time.time()
                # while True:
                #     #test sectuib
                #     if(time.time() -timer_test)>2:
                #         plt.plot(self.times,self.speeds)
                #         plt.show()
                #         exit(1)


                # #target = self.testloop()
                # angle_0, angle_1 = self.calculate_angles(target[0], target[1])
                # print(f"angles: {angle_0}, {angle_1}")
                # # print(timer)
                # # print(time.time())

                # # self.setgain(angle_0,angle_1)
                # # print(time.time())
                
                    
                # if self.armed:
                #     self.drive.goto_raw(angle_0, angle_1)
            except Exception as e:
                print(f"Exception: {e}")
                self.drive.set_idle()
                exit(1)

    def calculate_angles(self, target_pos_x, target_pos_y) -> float:
        x = (target_pos_x + self.x_offset) * self.x_multiplier
        y = (target_pos_y + self.y_offset) * self.y_multiplier
        pi = 4*math.atan2(1,1)
        pulley_diameter = 10.0  # cm
        distance_per_revolution = pi * pulley_diameter

        target_angle_0 = (y / distance_per_revolution) - (x / distance_per_revolution)
        target_angle_1 = (y / distance_per_revolution) + (x / distance_per_revolution)

        return target_angle_0, target_angle_1

    def at_exit(self):
        self.drive.set_idle()

    def calculateStrikerVelocity(self):
        # timer= time.time()
        while True:
            # print("calculate striker vel fps is "+ str(1/(time.time()-timer)))
            # timer= time.time()

            vel_0 = self.drive.drive.axis0.encoder.vel_estimate
            vel_1 = self.drive.drive.axis1.encoder.vel_estimate
            
            pi = 4*math.atan2(1,1)
            pulley_diameter = 0.1  # m, 10cm
            distance_per_revolution = pulley_diameter

            y = ((vel_1 + vel_0) * distance_per_revolution)
            x = ((vel_1 - vel_0) * distance_per_revolution)

            speed = (x**2 + y**2)**(1/2)
            # if speed < 1:
            #     continue
            # print(f"striker speed: {speed}")
            self.speeds.append(speed)
            self.times.append(time.time())
    
    #test loop to send fixed points to the odrive
    def testloop(self):
        time.sleep(0.2)
        print(f"iteration: {self.iteration}")
        target=[0, 0]
        target[0] = self.loopPoints[self.iteration*2]
        target[1] = self.loopPoints[self.iteration*2+1]
        self.iteration=self.iteration+1
        if(self.iteration==4):
            self.iteration=0
        return target

    #set gain on each motor to match the velocity curve in order to go in striaghter line
    def setgain(self, angle_0, angle_1):
        current_angle_0 = self.drive.drive.axis0.encoder.shadow_count/4000
        current_angle_1 = self.drive.drive.axis1.encoder.shadow_count/4000
        # print(abs(current_angle_0 - angle_0))
        # print(abs(current_angle_1 - angle_1))
        if self.gain_changed:
            self.drive.drive.axis0.controller.config.pos_gain = 20
            self.drive.drive.axis1.controller.config.pos_gain = 20
            self.gain_changed = False
        if current_angle_0 - angle_0 == 0 or current_angle_1 - angle_1 == 0:
            return
        else:    
            ratio = abs(current_angle_0 - angle_0) / abs(current_angle_1 - angle_1)
            # print(ratio)
            if ratio >= 1:
                # print("hello")
                self.drive.drive.axis0.controller.config.pos_gain = 20 * ratio**(1/2)
                self.drive.drive.axis1.controller.config.pos_gain = 20 / ratio**(1/2)
                self.gain_changed = True
            if ratio < 1:
                # print("hello2")
                self.drive.drive.axis0.controller.config.pos_gain = 20 * ratio**(1/2)
                self.drive.drive.axis1.controller.config.pos_gain = 20 / ratio**(1/2)
                self.gain_changed = True

    def send_to_odrive(self, in_q):
        timer = time.time()
        while True:
            # print("python fps is "+ str(1/(time.time()-timer)))
            timer = time.time()
            target = in_q.get()
            angle_0, angle_1 = self.calculate_angles(target[0], target[1])
            print(f"angles: {angle_0}, {angle_1}")
            if self.armed:
                self.drive.goto_raw(angle_0, angle_1)
            # time.sleep(0.01)

s = Server()