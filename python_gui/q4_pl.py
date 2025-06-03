##WRIST MATH

import math
import numpy as np # If you keep numpy for specific calculations
import config as cf

jaw_radius = 1.35 #mm

def get_steps_L(curr_theta, delta_theta):
    #radius = 1.35
    delta_s = math.radians(delta_theta)*jaw_radius
    steps = int(delta_s * (cf.STEPS_TO_MM_LS)) 
    
    motor_steps = [0] * len(cf.MotorIndex)
    motor_steps[cf.MotorIndex.LJL] = -steps
    motor_steps[cf.MotorIndex.LJR] = steps
    
    return motor_steps

def get_steps_R(curr_theta, delta_theta):
    #radius = 1.35
    delta_s = math.radians(delta_theta)*jaw_radius
    steps = int(delta_s * (cf.STEPS_TO_MM_LS)) 
    
    motor_steps = [0] * len(cf.MotorIndex)
    motor_steps[cf.MotorIndex.RJL] = steps
    motor_steps[cf.MotorIndex.RJR] = -steps
    return motor_steps
