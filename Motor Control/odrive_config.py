import odrive
from odrive.enums import *
import time
import socket
import atexit

class Odrive:
    def __init__(self) -> None:
        self.drive = odrive.find_any()
        time.sleep(1)
        # self.init_odrive()
        self.set_zero()
        self.set_closed_loop()

    # def init_odrive(self) -> None:
    #     self.drive.axis0.controller.config.input_mode = 1
    #     self.drive.axis1.controller.config.input_mode = 1
    #     self.drive.axis0.motor.config.current_lim = 70
    #     self.drive.axis1.motor.config.current_lim = 70
    #     self.drive.axis0.motor.config.current_lim_margin = 30
    #     self.drive.axis1.motor.config.current_lim_margin = 30
    #     self.drive.axis0.controller.config.vel_limit = 10
    #     self.drive.axis1.controller.config.vel_limit = 10
    #     self.drive.axis0.controller.config.pos_gain = 20  # 30
    #     self.drive.axis1.controller.config.pos_gain = 20  # 30
    #     print(f"inited")
            

    def config_odrive(self) -> None:
        self.drive.axis0.config.startup_encoder_offset_calibration = True   #   encoder calibration on startup (forward and backward rotation of both motors on startup)
        self.drive.axis1.config.startup_encoder_offset_calibration = True

        self.drive.axis0.encoder.config.mode = 0  # ENCODER_MODE_HALL This one doesn't work...
        self.drive.axis1.encoder.config.mode = 0  # ENCODER_MODE_HALL This one doesn't work...
        self.drive.axis0.encoder.config.cpr = 4000
        self.drive.axis1.encoder.config.cpr = 4000

        self.drive.axis0.encoder.config.use_index = False
        self.drive.axis1.encoder.config.use_index = False
        self.drive.axis0.encoder.config.ignore_illegal_hall_state = True
        self.drive.axis1.encoder.config.ignore_illegal_hall_state = True

        self.drive.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        self.drive.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        
        self.drive.axis0.motor.config.requested_current_range = 70
        self.drive.axis1.motor.config.requested_current_range = 70 
        
        # self.drive.axis1.encoder.config.bandwidth = 100

        self.drive.save_configuration()
        # self.drive.reboot()

        print(f"Success! Xinyu why?")
        time.sleep(5)

        self.drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        # time.sleep(5)
        self.drive.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        #time.sleep(5)
        print(f"CALIBRATING")

        time.sleep(5)
        if(self.drive.axis0.encoder.error!=0):
            print(f"Error: Oodrv0 reported an error of {self.drive.axis0.encoder.error} while in the state ")

    def set_zero(self) -> None:
        self.drive.axis0.encoder.set_linear_count(0)
        self.drive.axis1.encoder.set_linear_count(0)

    def set_idle(self) -> None:
        self.drive.axis0.requested_state = AXIS_STATE_IDLE
        self.drive.axis1.requested_state = AXIS_STATE_IDLE

    def set_closed_loop(self) -> None:
        self.drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(1)

    def goto_raw(self, mot_0, mot_1) -> None:
        self.drive.axis0.controller.input_pos = mot_0
        self.drive.axis1.controller.input_pos = mot_1


o = Odrive()
o.config_odrive()