##ELBOW PITCH MATH
##VERSION: 1.15

import math
import config
from config import MotorIndex, STEPS_TO_MM_LS, Q1_DR_COMP, DIR_COMP, ls_steps_from_mm
from kinematic_model import get_q3_pl


dir_offset = Q1_DR_COMP

#geometric constants:
q1_rad = 1.5 #mm
# Positive step values (or positive delta_theta) = cable shortening

def get_jaw_pl(delta_theta):
    r2 = 1.25
    delta_s = math.radians(delta_theta)*r2
    return delta_s

def get_q3_pl(curr_theta, delta_theta):
    ###THIS IS A SIMPLIFIED VERSION THAT HOLDS TRUE IF THETA BETWEEN THE BOUNDARY ANGLE AND THE BOUNDARY ANGLE + 90 DEG
    ##IMPLEMENTING FOR INITIAL TESTING ON JUNE 20
    r2 = 1.5
    delta_q3_pos = math.radians(delta_theta)*r2
    delta_q3_neg = math.radians(delta_theta)*r2

    steps_q3_pos = ls_steps_from_mm(delta_q3_pos)
    steps_q3_neg = ls_steps_from_mm(delta_q3_neg)

    """
    Uncomment this code block when Ali updates Kinematic_model.py
    q3_pos_curr, q3_pos_neg = get_q3_pl(curr_theta)
    q3_pos_final, q3_neg_final = get_q3_pl(curr_theta + delta_theta)

    delta_q3_pos = q3_pos_final - q3_pos_curr
    delta_q3_neg = q3_neg_final - q3_pos_neg
    """

    return steps_q3_pos, steps_q3_neg


def get_steps(curr_theta, delta_theta, latest_dir):
    motor_steps = [0] * len(MotorIndex)

    if delta_theta == 0:
        return motor_steps, latest_dir
    
    mm_q1 = math.radians(delta_theta)*q1_rad
    print("Calculated required Q1 path length change to be:", mm_q1)
    steps_q1 = int(mm_q1*STEPS_TO_MM_CAPSTAN)

    ep_comp = 0
    
    """
    Directional hysteresis compensation logic: 
    if (latest_dir == 0 or latest_dir*delta_theta < 0 and DIR_COMP):
        ###latest_dir and delta_theta are not the same direction/sign
        ###have to add compensation and swap direction
        ep_comp = math.copysign(dir_offset, steps_q1)
        if(latest_dir == 0):
            ep_comp = ep_comp/2
        print(f"Added {ep_comp} steps due to a change in direction! new latest dir: {delta_theta} last dir: {latest_dir}")
        latest_dir = delta_theta
    else:
        print(f"latest_dir didnt change or dir_comp is off: latest_dir = {latest_dir} delta theta = {delta_theta} and dr comp = {DIR_COMP}")
        """

    #OVERALL, FOR COMPENSATION, EPU, LJL, LJR, WPD must be the same sign

    #Positive steps moves Q1 UP 

    motor_steps = [0] * len(MotorIndex)  
    motor_steps[MotorIndex.EP] = steps_q1+ep_comp ##positive step value here will move EP DOWN , so we want WPD , RJR RJL to lengthen (positive)

    #get aux steps
    q3_pos_steps, q3_neg_steps = int(get_q3_pl(delta_theta)*(STEPS_TO_MM_LS))
    steps_q4 = int(get_jaw_pl(delta_theta)*STEPS_TO_MM_LS)
    
    motor_steps[MotorIndex.LJL] = -steps_q4 
    motor_steps[MotorIndex.LJR] = -steps_q4


    motor_steps[MotorIndex.RJL] = steps_q4
    motor_steps[MotorIndex.RJR] = steps_q4


    motor_steps[MotorIndex.WPU] = steps_q3
    motor_steps[MotorIndex.WPD] = -steps_q3
    
    return motor_steps, latest_dir

