##ELBOW PITCH MATH
##VERSION: 1.15

import math
from config import MotorIndex, STEPS_TO_MM_LS, STEPS_TO_MM_CAPSTAN, Q1_DR_COMP, ls_steps_from_mm
from kinematic_model import get_q3_pl
from experimental_model import get_q3_change


dir_offset = Q1_DR_COMP

#geometric constants:
q1_rad = 1.5 #mm
# Positive step values (or positive delta_theta) = cable shortening

def get_jaw_pl(delta_theta):
    r2 = 1.25
    delta_s = math.radians(delta_theta)*r2
    return delta_s

def get_q3_pl(curr_theta, delta_theta):

    q3_radius = 1.7 #mm
    curr_theta_rad = math.radians(curr_theta)
    delta_theta_rad = math.radians(delta_theta)
    angle_comp_rad = -get_q3_change(curr_theta_rad, delta_theta_rad)
    mm_comp = angle_comp_rad*q3_radius

    ##a positive predicted change means the pitch will move down, so we must move it up 

    delta_q3_pos = -mm_comp
    delta_q3_neg = mm_comp

    steps_q3_pos = ls_steps_from_mm(delta_q3_pos)
    steps_q3_neg = ls_steps_from_mm(delta_q3_neg)

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
    steps_q3_pos, steps_q3_neg = get_q3_pl(curr_theta, delta_theta)
    steps_q4 = int(get_jaw_pl(delta_theta)*STEPS_TO_MM_LS)
    
    motor_steps[MotorIndex.LJL] = -steps_q4 
    motor_steps[MotorIndex.LJR] = -steps_q4


    motor_steps[MotorIndex.RJL] = steps_q4
    motor_steps[MotorIndex.RJR] = steps_q4


    motor_steps[MotorIndex.WPU] = steps_q3_neg
    motor_steps[MotorIndex.WPD] = steps_q3_pos
    
    return motor_steps, latest_dir

def sanity_check():
    """
    Runs a series of tests on the get_steps function to verify its output
    for both positive and negative commands.
    """
    test_angles = [10.3, -5]  # Test both directions
    curr_theta = 90.0      # Assume a neutral starting angle
    latest_dir = 0         # Assume no previous movement

    print("--- Running Sanity Check for Elbow Pitch (Q1) ---")
    print("Expected -- positive EP means positive WPD and RJs")

    for delta_theta in test_angles:
        # Calculate the motor steps required for the angle change
        steps, latest_dir = get_steps(curr_theta, delta_theta, latest_dir)
        
        print(f"\nCommanding {delta_theta}° change from {curr_theta}°:")
        
        # Iterate directly over the MotorIndex enum to ensure accurate labeling
        for motor in MotorIndex:
            # motor.name provides the string name (e.g., "EP")
            # motor.value provides the integer index (e.g., 6)
            print(f"{motor.name}: {steps[motor.value]}")

#sanity_check()