##ELBOW YAW MATH
##VERSION: 1.15

import math
import numpy as np 
import config as cf

EY_effective_radius = 1.4 ##mm


def get_aux_pl(theta_deg):
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

def get_steps(curr_theta, delta_theta):
    #curr theta is in degrees between 0 and 180

    delta_ey = math.radians(delta_theta) * EY_effective_radius
    steps_ey = int(delta_ey*(cf.STEPS_TO_MM_LS))

    motor_steps = [0] * len(cf.MotorIndex)
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

    return motor_steps