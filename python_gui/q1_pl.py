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
    motor_steps = [0] * len(MotorIndex)  
    if delta_theta == 0:
        return motor_steps, latest_dir
    
    mm_q1 = math.radians(delta_theta)*radius
    steps_q1 = int(mm_q1*cf.STEPS_TO_MM_LS)

    target_theta = curr_theta + delta_theta
    
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

    motor_steps = [0] * len(MotorIndex)  
    motor_steps[MotorIndex.EPU] = -steps_q1+epu_comp
    motor_steps[MotorIndex.EPD] = steps_q1+epd_comp

    #get aux steps
    steps_q3 = int(get_wrist_pl(delta_theta)*(STEPS_TO_MM_LS))
    steps_q4 = int(get_jaw_pl(delta_theta)*STEPS_TO_MM_LS)
    
    motor_steps[MotorIndex.LJL] = -steps_q4 
    motor_steps[MotorIndex.LJR] = -steps_q4


    motor_steps[MotorIndex.RJL] = steps_q4
    motor_steps[MotorIndex.RJR] = steps_q4


    motor_steps[MotorIndex.WPU] = -steps_q3
    motor_steps[MotorIndex.WPD] = steps_q3
    
    return motor_steps, latest_dir


def plot_epu_epd_vs_q1():
    q1_range = np.linspace(0, 180, 181)
    epd_list = []
    epu_list = []
    for theta in q1_range:
        epd, epu = get_pl(theta)
        epd_list.append(epd)
        epu_list.append(epu)
    plt.figure(figsize=(8, 5))
    plt.plot(q1_range, epd_list, label='EPD Length')
    plt.plot(q1_range, epu_list, label='EPU Length')
    plt.xlabel('q1 (degrees)')
    plt.ylabel('Cable Length (units)')
    plt.title('EPD and EPU Lengths vs q1')
    plt.legend()
    plt.grid(True)
    plt.savefig("python_gui/epu_epd_lengths_vs_q1.png")
    plt.show()


def plot_aux_vs_q1():
    q1_range = np.linspace(0, 180, 181)
    aux_up_left = []
    aux_down_right = []
    for theta in q1_range:
        aux_up_left.append(get_aux_pl(theta))
        aux_down_right.append(get_aux_pl(180 - theta))
    plt.figure(figsize=(8, 5))
    plt.plot(q1_range, aux_up_left, label='Aux Up Left')
    plt.plot(q1_range, aux_down_right, label='Aux Down Right')
    plt.xlabel('q1 (degrees)')
    plt.ylabel('Aux Cable Length (units)')
    plt.title('Aux Cable Lengths vs q1')
    plt.legend()
    plt.grid(True)
    plt.savefig("python_gui/aux_cable_lengths_vs_q1.png")
    plt.show()


