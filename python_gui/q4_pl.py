##WRIST MATH

import math
import numpy as np # If you keep numpy for specific calculations
import config as cf

jaw_radius = 1.35 #mm


def get_steps_L(curr_theta, delta_theta, latest_dir):
    dir_offset = cf.Q4_L_DR_COMP
    motor_steps = [0] * len(cf.MotorIndex)

    if(delta_theta == 0):
        return motor_steps, latest_dir
    
    #radius = 1.35
    delta_s = math.radians(delta_theta)*jaw_radius
    steps = int(delta_s * (cf.STEPS_TO_MM_LS)) 

    if (latest_dir == 0 or latest_dir*delta_theta < 0 and cf.DIR_COMP):
        ###latest_dir and delta_theta are not the same direction/sign
        ###have to add compensation and swap direction
        comp = math.copysign(dir_offset, steps)
        if(latest_dir == 0):
            comp = comp/2
        steps += comp
        latest_dir = delta_theta
        print(f"Added {comp} steps due to a change in direction! new latest dir: {latest_dir}")
    else:

        print(f"latest_dir didnt change or dir_comp is off: latest_dir = {latest_dir} delta theta = {delta_theta} and dr comp = {cf.DIR_COMP}")
        print("oopsies")
    
    motor_steps[cf.MotorIndex.LJL] = steps
    motor_steps[cf.MotorIndex.LJR] = -steps
    
    return motor_steps, latest_dir

def get_steps_R(curr_theta, delta_theta, latest_dir):
    #radius = 1.35
    dir_offset = cf.Q4_R_DR_COMP
    motor_steps = [0] * len(cf.MotorIndex)

    if(delta_theta == 0):
        return motor_steps, latest_dir
    
    delta_s = math.radians(delta_theta)*jaw_radius
    steps = int(delta_s * (cf.STEPS_TO_MM_LS)) 

    if (latest_dir == 0 or latest_dir*delta_theta < 0 and cf.DIR_COMP):
        ###latest_dir and delta_theta are not the same direction/sign
        ###have to add compensation and swap direction
        comp = math.copysign(dir_offset, steps)
        if(latest_dir == 0):
            comp = comp/2
        
        steps += comp
        latest_dir = delta_theta
        print(f"Added {comp} steps due to a change in direction! new latest dir: {latest_dir}")
    else:

        print(f"latest_dir didnt change or dir_comp is off: latest_dir = {latest_dir} delta theta = {delta_theta} and dr comp = {cf.DIR_COMP}")
        print("oopsies")
    
    motor_steps[cf.MotorIndex.RJL] = -steps
    motor_steps[cf.MotorIndex.RJR] = steps
    return motor_steps, latest_dir
