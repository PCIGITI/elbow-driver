##ELBOW PITCH MATH
##VERSION: 1.15

import math
import numpy as np 
import config as cf
from config import MotorIndex, STEPS_TO_MM_LS
import matplotlib.pyplot as plt

dir_offset = cf.Q1_DR_COMP

def get_aux_pl(theta_deg):
    ax = 0.75
    ay = 0.6
    q1_rad = math.radians(theta_deg)

    bx = 0.48
    by = -3.3

    ax_rotated = ax * np.cos(q1_rad) - ay * np.sin(q1_rad)
    ay_rotated = ax * np.sin(q1_rad) + ay * np.cos(q1_rad)

    ##approximates wrist pl as a straight line :3
    len = np.sqrt((bx - ax_rotated)**2 + (by - ay_rotated)**2)

    return len

def get_pl(theta_deg):

    true_theta_deg = theta_deg
    if theta_deg < 90:
        theta_deg = 180-theta_deg
    # A = cable entrance at the side
    ax_at_90_deg = -2.5 
    ay_at_90_deg = 1.83

    # B = left cable entrance
    bx = -1.43
    by = -3.3

    # Base coordinates of point A (at 0-degree rotation of q2)
    x_base_A = ay_at_90_deg  
    y_base_A = -ax_at_90_deg 

    # --- Setup for Point C and Point D ---
    # Coordinates of point C when its rotation angle q2 is 90 degrees.
    cx_at_90_deg = 1.75
    cy_at_90_deg = 0.25

    # Coordinates of the fixed point D
    dx = 1.43
    dy = -3.3 # Note: Same y-coordinate as point B, x-coordinate is different.

    # Constant h for the 'longer cable'
    h_constant = 1.7

    # Base coordinates of point C (at 0-degree rotation of q2)
    x_base_C = cy_at_90_deg   # So, x_base_C = 0.25
    y_base_C = -cx_at_90_deg  # So, y_base_C = -1.75

    # --- Generate rotation angles for q2 ---
    # Angles q2 from 90 to 130 degrees. Using 100 points for a smooth curve.
    q1_rad = math.radians(theta_deg)

    # --- Calculate A's rotated coordinates and distance to B ---
    ax_rotated = x_base_A * np.cos(q1_rad) - y_base_A * np.sin(q1_rad)
    ay_rotated = x_base_A * np.sin(q1_rad) + y_base_A * np.cos(q1_rad)
    shorter = np.sqrt((bx - ax_rotated)**2 + (by - ay_rotated)**2)

    # --- Calculate C's rotated coordinates, distance to D, and 'longer cable' length ---
    cx_rotated = x_base_C * np.cos(q1_rad) - y_base_C * np.sin(q1_rad)
    cy_rotated = x_base_C * np.sin(q1_rad) + y_base_C * np.cos(q1_rad)
    
    # Verify coordinates of C at q2 = 90 degrees
    # print(f"Calculated C at q2=90: ({cx_rotated[0]:.2f}, {cy_rotated[0]:.2f})") # Should be (1.75, 0.25)

    longer = np.sqrt((dx - cx_rotated)**2 + (dy - cy_rotated)**2)
    longer = h_constant + longer

    print(f"Calculated EP lengths for theta {theta_deg} degrees: shorter={shorter:.2f}, longer={longer:.2f}")  # Debug output

    if true_theta_deg >= 90:  #then pitch down will be shorter!!
        epd, epu = shorter, longer  
    else:
        epu, epd = shorter, longer  

    return epd, epu

def get_steps(curr_theta, delta_theta, latest_dir):
    motor_steps = [0] * len(MotorIndex)  
    if delta_theta == 0:
        return motor_steps, latest_dir
    target_theta = curr_theta + delta_theta

    curr_epd, curr_epu = get_pl(curr_theta)
    target_epd, target_epu = get_pl(target_theta)

    print(f"calculated delta epu: {target_epu - curr_epu}, delta epd: {target_epd - curr_epd} for theta {target_theta} from {curr_theta}")  # Debug output
    
    delta_epu = target_epu - curr_epu
    delta_epd = target_epd - curr_epd

    steps_epu = -int(delta_epu*(STEPS_TO_MM_LS))
    steps_epd = -int(delta_epd*(STEPS_TO_MM_LS))
    epd_comp = 0
    epu_comp = 0

    if (latest_dir == 0 or latest_dir*delta_theta < 0 and cf.DIR_COMP):
        ###latest_dir and delta_theta are not the same direction/sign
        ###have to add compensation and swap direction
        epu_comp = math.copysign(dir_offset, steps_epu)
        epd_comp = math.copysign(dir_offset, steps_epd)
        if(latest_dir == 0):
            epd_comp = epd_comp/2  
            epu_comp = epu_comp/2      
        print(f"Added {epd_comp} steps due to a change in direction! new latest dir: {delta_theta} last dir: {latest_dir}")
        latest_dir = delta_theta
    else:

        print(f"latest_dir didnt change or dir_comp is off: latest_dir = {latest_dir} delta theta = {delta_theta} and dr comp = {cf.DIR_COMP}")
        print("oopsies")

    motor_steps = [0] * len(MotorIndex)  
    motor_steps[MotorIndex.EPU] = steps_epu+epu_comp
    motor_steps[MotorIndex.EPD] = steps_epd+epd_comp

    #get aux steps
    curr_aux_down_right = get_aux_pl(180-curr_theta) 
    curr_aux_up_left = get_aux_pl(curr_theta)
    target_aux_down_right = get_aux_pl(180-target_theta) 
    target_aux_up_left = get_aux_pl(target_theta) 

    delta_aux_UL = target_aux_up_left-curr_aux_up_left
    delta_aux_DR = target_aux_down_right-curr_aux_down_right

    steps_aux_UL = -int(delta_aux_UL*(STEPS_TO_MM_LS))
    steps_aux_DR = -int(delta_aux_DR*(STEPS_TO_MM_LS))
    motor_steps[MotorIndex.LJL] = steps_aux_DR 
    motor_steps[MotorIndex.RJL] = steps_aux_UL
    motor_steps[MotorIndex.WPU] = steps_aux_UL
    motor_steps[MotorIndex.WPD] = steps_aux_DR
    motor_steps[MotorIndex.RJR] = steps_aux_UL
    motor_steps[MotorIndex.LJR] = steps_aux_DR 
    
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


