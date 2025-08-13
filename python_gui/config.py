# config.py
from enum import IntEnum
import serial # For serial constants if needed
#--- JOINT KINEMATICS ---

''' 

Ground truth for sign conventions: 
Q1 = Elbow Pitch
Positive: Down

Q2 = Elbow Yaw
Positive: Right

Q3 = Wrist Pitch
Positive: Down

Q4R = Right Jaw
Positive: Close (Left)

Q4L = Left Jaw
Positive: Close (Right)

For current dynamixel setup (Aug 12, 2025):

MOTOR  ||   STEPS   ||  DIRECTION
Q1          +ve         DOWN
Q2          +ve         RIGHT

For motors on lead screws, positive steps lengthen the cable. Negative steps shorten the cable.
***THIS MEANS SIGN CONVENTIONS ARE FLIPPED FOR Q1 Q2 VS Q3 Q4: ***
POSITIVE STEPS INTO Q1 = POSITIVE CHANGE IN Q1
NEGATIVE STEPS INTO Q3+ = POSITIVE CHANGE IN Q3


WPU -> Q3-
WPD -> Q3+

RJR -> Q4R-
RJL -> Q4R+

LJR -> Q4L+
LJL -> Q4L-

We'll talk to the dynamixel driver (Open RB 150) using the following order:

[Q1, Q2, WPD (Q3+), WPU (Q3-), RJL (Q4R+), RJR (Q4R-), LJR (Q4L+), LJL (Q4L-)]
'''


# --- MOTOR DEFINITIONS ---
class MotorIndex(IntEnum):
    EP = 0
    EY = 1
    WPD = 2
    WPU = 3
    RJL = 4
    RJR = 5
    LJR = 6
    LJL = 7


# --- MOVEMENT PARAMETERS ---
STEP_SIZES = [2, 5, 10, 15, 20, 25, 30, 100, 200, 300, 500, 800]
DEFAULT_STEP_SIZE_INDEX = STEP_SIZES.index(10)
MOTOR_TEST_DEFAULT_STEP_COUNT = 50
MOTOR_TEST_STEP_MIN = -200
MOTOR_TEST_STEP_MAX = 200

# --- SERIAL COMMUNICATION ---
DEFAULT_SERIAL_PORT = "COM8"
SERIAL_BAUDRATE = 9600
SERIAL_BYTESIZE = serial.EIGHTBITS
SERIAL_PARITY = serial.PARITY_NONE
SERIAL_STOPBITS = serial.STOPBITS_ONE
SERIAL_TIMEOUT = 1
SERIAL_CONNECT_DELAY = 2 # Time to wait for Arduino reset

# --- CONVERSION FACTORS ---
STEPS_TO_MM_LS = 4096 / 0.3 # (steps_per_rev / mm_per_rev)
STEPS_TO_MM_CAPSTAN = 4096 / 15.7 # (steps_per_rev / capstan circumfrence)
# --- DIRECTION CHANGE FACTORS (steps)

DIR_COMP = False
#These numbers are old and if you want to use direction compensation must be updated for the new miniaturized setup. 
Q1_DR_COMP = 125 
Q2_DR_COMP = 103.58
Q3_DR_COMP = 132.3
Q4_L_DR_COMP = 271
Q4_R_DR_COMP = 558

# --- UI ---
APP_TITLE = "Elbow Control Simulator"
MAIN_WINDOW_GEOMETRY = "950x950"
TEST_MOTORS_WINDOW_GEOMETRY = "750x450"
