##ELBOW YAW MATH
##VERSION: 1.15


##LIMITATIONS: NEUTRAL +- 50 DEGREES!
##THETA ABODE 140 OR BELOW 40 IS NOT VALID!

import math
import numpy as np 
import config as cf
from config import MotorIndex, STEPS_TO_MM_LS, ls_steps_from_mm
import matplotlib.pyplot as plt

dir_offset = cf.Q2_DR_COMP
EY_effective_radius = 1.3 ##mm

def get_jaw_pl(curr_theta, delta_theta):
    r2 = 1.3

    ###THIS IS A SIMPLIFIED VERSION THAT HOLDS TRUE IF THETA BETWEEN THE BOUNDARY ANGLE AND THE BOUNDARY ANGLE + 90 DEG
    ##IMPLEMENTING FOR INITIAL TESTING ON JUNE 20
    delta_q4_pos = math.radians(delta_theta)*r2
    delta_q4_neg = math.radians(delta_theta)*r2

    steps_q4_pos = ls_steps_from_mm(delta_q4_pos)
    steps_q4_neg = ls_steps_from_mm(delta_q4_neg)

    """
    Uncomment this code block when Ali updates Kinematic_model.py
    q4_pos_curr, q4_pos_neg = get_q4_pl(curr_theta)
    q4_pos_final, q4_neg_final = get_q4_pl(curr_theta + delta_theta)

    delta_q4_pos = q4_pos_final - q4_pos_curr
    delta_q4_neg = q4_neg_final - q4_pos_neg
    """

    return steps_q4_pos, steps_q4_neg


def get_steps(curr_theta, delta_theta, latest_dir):
    print("aug 14 11:04am");
    motor_steps = [0] * len(cf.MotorIndex)  
    if delta_theta == 0:
        return motor_steps, latest_dir
    #curr theta is in degrees between 0 and 180

    delta_ey = math.radians(delta_theta)*EY_effective_radius
    print("calculated required path length change to be:", delta_ey)
    steps_ey = int(delta_ey*(cf.STEPS_TO_MM_CAPSTAN))

    if (latest_dir == 0 or latest_dir*delta_theta < 0 and cf.DIR_COMP):
        ###latest_dir and delta_theta are not the same direction/sign
        ###have to add compensation and swap direction
        comp = math.copysign(dir_offset, steps_ey)
        if(latest_dir == 0):
            comp = comp/2
        steps_ey += comp
        latest_dir = delta_theta
        print(f"Added {comp} steps due to a change in direction! new latest dir: {latest_dir}")
    else:

        print(f"latest_dir didnt change or dir_comp is off: latest_dir = {latest_dir} delta theta = {delta_theta} and dr comp = {cf.DIR_COMP}")
        print("oopsies")

    ## positive ey steps moves us to the left, so we have to shortehn RJR and LJR 
    ## positive steps_ey --> negative steps RJR LJR

    motor_steps[MotorIndex.EY] = steps_ey

    steps_q4_pos, steps_q4_neg = get_jaw_pl(curr_theta, delta_theta)

    motor_steps[MotorIndex.RJL] = steps_q4_pos
    motor_steps[MotorIndex.LJL] = steps_q4_pos
    motor_steps[MotorIndex.RJR] = steps_q4_neg
    motor_steps[MotorIndex.LJR] = steps_q4_neg

    # [Q1, Q2, WPD (Q3+), WPU (Q3-), RJL (Q4R+), RJR (Q4R-), LJR (Q4L+), LJL (Q4L-)]


    """ motor_steps[MotorIndex.RJL] = -steps_q4
    motor_steps[MotorIndex.LJL] = -steps_q4
    motor_steps[MotorIndex.RJR] = steps_q4
    motor_steps[MotorIndex.LJR] = steps_q4
    """

    return motor_steps, latest_dir
