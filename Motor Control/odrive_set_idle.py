import odrive
from odrive.enums import *
import time
import socket
import atexit
import sys

drive = odrive.find_any()
drive.axis0.requested_state = AXIS_STATE_IDLE
drive.axis1.requested_state = AXIS_STATE_IDLE
print(f"Set Idle Complete, Press CTRL + C to exit program.")