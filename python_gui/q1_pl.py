##ELBOW PITCH MATH
##VERSION: 1.15

import math
import numpy as np 
import config as cf
from config import MotorIndex, STEPS_TO_MM_LS
import matplotlib.pyplot as plt

dir_offset = cf.Q1_DR_COMP

#geometric constants:
radius = 1.5 #mm
# Positive step values (or positive delta_theta) = cable shortening

def get_jaw_pl(delta_theta):
    r2 = 1.25
    delta_s = math.radians(delta_theta)*r2

    return delta_s

def get_wrist_pl(delta_theta):
    ###THIS IS A SIMPLIFIED VERSION THAT HOLDS TRUE IF THETA BETWEEN THE BOUNDARY ANGLE AND THE BOUNDARY ANGLE + 90 DEG
    ##IMPLEMENTING FOR INITIAL TESTING ON JUNE 20
    r2 = 1.5
    delta_s = math.radians(delta_theta)*r2

    return delta_s

def get_steps(curr_theta, delta_theta, latest_dir):
    print(curr_theta)    


    motor_steps = [0] * len(MotorIndex)  
    if delta_theta == 0:
        return motor_steps, latest_dir
    
    mm_q1 = math.radians(delta_theta)*radius
    print("calculated required path length change to be:", mm_q1)
    steps_q1 = int(mm_q1*cf.STEPS_TO_MM_CAPSTAN)
    
    epd_comp = 0
    epu_comp = 0

    if (latest_dir == 0 or latest_dir*delta_theta < 0 and cf.DIR_COMP):
        ###latest_dir and delta_theta are not the same direction/sign
        ###have to add compensation and swap direction
        epu_comp = math.copysign(dir_offset, steps_q1)
        epd_comp = math.copysign(dir_offset, -steps_q1)
        if(latest_dir == 0):
            epd_comp = epd_comp/2  
            epu_comp = epu_comp/2      
        print(f"Added {epd_comp} steps due to a change in direction! new latest dir: {delta_theta} last dir: {latest_dir}")
        latest_dir = delta_theta
    else:
        print(f"latest_dir didnt change or dir_comp is off: latest_dir = {latest_dir} delta theta = {delta_theta} and dr comp = {cf.DIR_COMP}")


    #OVERALL, FOR COMPENSATION, EPU, LJL, LJR, WPD must be the same sign

    #Positive steps moves Q1 UP 

    motor_steps = [0] * len(MotorIndex)  
    motor_steps[MotorIndex.EP] = steps_q1+epu_comp

    #get aux steps
    steps_q3 = int(get_wrist_pl(delta_theta)*(STEPS_TO_MM_LS))
    steps_q4 = int(get_jaw_pl(delta_theta)*STEPS_TO_MM_LS)
    
    motor_steps[MotorIndex.LJL] = steps_q4 
    motor_steps[MotorIndex.LJR] = steps_q4


    motor_steps[MotorIndex.RJL] = -steps_q4
    motor_steps[MotorIndex.RJR] = -steps_q4


    motor_steps[MotorIndex.WPU] = -steps_q3
    motor_steps[MotorIndex.WPD] = steps_q3
    
    return motor_steps, latest_dir

