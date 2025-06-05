##ELBOW YAW MATH
##VERSION: 1.15


##LIMITATIONS: NEUTRAL +- 50 DEGREES!
##THETA ABODE 140 OR BELOW 40 IS NOT VALID!

import math
import numpy as np 
import config as cf
import matplotlib.pyplot as plt

dir_offset = cf.Q2_DR_COMP


EY_effective_radius = 1.4 ##mm

def get_aux_pl(theta_deg):
    true_theta_deg = theta_deg
    if theta_deg < 90:
        theta_deg = 180-theta_deg
    # A = cable entrance at the side
    ax_at_90_deg = -1.1 
    ay_at_90_deg = 0.4

    # B = left cable entrance
    bx = -1.9
    by = -1.1

    # Base coordinates of point A (at 0-degree rotation of q2)
    x_base_A = ay_at_90_deg  
    y_base_A = -ax_at_90_deg 

    # --- Setup for Point C and Point D ---
    # Coordinates of point C when its rotation angle q2 is 90 degrees.
    cx_at_90_deg = 1.1
    cy_at_90_deg = 0

    # Coordinates of the fixed point D
    dx = 1.9
    dy = -1.1 # Note: Same y-coordinate as point B, x-coordinate is different.

    # Constant h for the 'longer cable'
    h_constant =0.34

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
        pl_jl, pl_jr = shorter, longer  
    else:
        pl_jr, pl_jl = shorter, longer  

    return pl_jl, pl_jr

def old_get_aux_pl(theta_deg): #DEPRECIATED
    rc = 1.56 / 2
    rs = 0.5
    l_1 = math.hypot(1.4, 1.9) # math.sqrt(1.4**2 + 1.9**2)

    if theta_deg >= 90:
        theta = math.radians(180 - theta_deg)
    else:
        theta = math.radians(theta_deg)
    
    x_a = (rc / 2) * (math.cos(theta - 0.872)) - 1.4
    y_a = (rc / 2) * (math.sin(theta - 0.872)) + 1.9
    
    h = math.hypot(x_a, y_a)
    len_line = math.hypot(h, rs)

    beta = math.atan(1.9 / -1.4) - math.atan(y_a / x_a)
    
    lambda_angle = math.asin((math.sin(beta) * l_1) / rc)
    
    alpha = math.asin(rs / len_line) - lambda_angle
    
    arc_length = alpha * rs
    
    shorter = arc_length + len_line + 0.1947
    longer = 1.1 * (1.5708 - theta) + 2.5099

    if theta_deg >= 90:  # then jaw left will be shorter !!
        pl_jl, pl_jr = shorter, longer  # Corrected
    else:  # then jaw right will be shorter !!
        pl_jr, pl_jl = shorter, longer


    return pl_jl, pl_jr

def get_steps(curr_theta, delta_theta, latest_dir):
    motor_steps = [0] * len(cf.MotorIndex)  
    if delta_theta == 0:
        return motor_steps, latest_dir
    #curr theta is in degrees between 0 and 180

    delta_ey = math.radians(delta_theta) * EY_effective_radius
    steps_ey = int(delta_ey*(cf.STEPS_TO_MM_LS))


    if(curr_theta + delta_theta > 140 or curr_theta + delta_theta < 40):
        print("ERROR: Elbow yaw angle out of range (40 to 140 degrees). Error thrown with delta_theta: ", delta_theta)
        return motor_steps
    

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


    motor_steps[cf.MotorIndex.EYR] = -steps_ey
    motor_steps[cf.MotorIndex.EYL] = steps_ey

    

    #get aux steps: wrist pitch is unbothered. when elbow yaw ++ (goes left). jaw left cables shorten, jaw right cables lengthen. 
    #only valid within 32 degrees of rom
    
    target_theta = curr_theta + delta_theta
    curr_jl, curr_jr = get_aux_pl(curr_theta)  
    target_jl, target_jr = get_aux_pl(target_theta)  

    delta_jl = target_jl - curr_jl
    delta_jr = target_jr - curr_jr

    steps_jl = -int(delta_jl*(cf.STEPS_TO_MM_LS))
    steps_jr = -int(delta_jr*(cf.STEPS_TO_MM_LS))

    motor_steps[cf.MotorIndex.RJL] = steps_jl
    motor_steps[cf.MotorIndex.LJR] = steps_jr
    motor_steps[cf.MotorIndex.LJL] = steps_jl
    motor_steps[cf.MotorIndex.RJR] = steps_jr

    return motor_steps, latest_dir


def plot_aux_cable_lengths():
    q2_range = np.linspace(40, 140, 361)
    jl_lengths = []
    jr_lengths = []

    for theta in q2_range:
        pl_jl, pl_jr = get_aux_pl(theta)
        jl_lengths.append(pl_jl)
        jr_lengths.append(pl_jr)

    plt.figure(figsize=(8, 5))
    plt.plot(q2_range, jl_lengths, label='Jaw Left Cable Length')
    plt.plot(q2_range, jr_lengths, label='Jaw Right Cable Length')
    plt.xlabel('q2 (degrees)')
    plt.ylabel('Cable Length (mm)')
    plt.title('Aux Cable Lengths vs q2')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('aux_cable_lengths_q2.png')
    plt.show()
