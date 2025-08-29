import math
import numpy as np
import matplotlib.pyplot as plt


def get_q3_pl(q1_joint_angle_degrees):
    """
    Calculate cable lengths for joint 3 based on joint 1 angle.
    
    Args:
        q1_joint_angle_degrees (float): Joint 1 angle in degrees
        
    Returns:
        tuple: (q3_pos, q3_neg) - Cable lengths in mm
    """
    # Physical constants (converted from MATLAB)
    L_AE = 1e-3 * 1.91
    L_BC = 1e-3 * 1.09
    r_PD = 1e-3 * 1.5
    r_OA = 1e-3 * 1.25
    a_lag_q = 66.92
    b_lag_horizontal = 25.83
    CD_angle = 25.83
    OM = 1e-3 * 2.25
    OE = 1e-3 * 2.29
    ME = 1e-3 * 0.4
    Px = -1e-3 * 2.18
    Py = 1e-3 * 2
    
    # Derived constants
    SCD_min = r_PD * CD_angle * math.pi / 180
    l_pos_neg_comp = 0
    l_base = SCD_min + L_BC + L_AE
    q_rad = q1_joint_angle_degrees * (math.pi / 180)
    q_switch_deg = a_lag_q - b_lag_horizontal
    
    # Calculate geometric parameters
    Ex = OE * math.sin(q_rad - math.atan2(ME, OM))
    Ey = OE * math.cos(q_rad - math.atan2(ME, OM))
    sqrt_arg = (Ex - Px)**2 + (Ey - Py)**2 - r_PD**2
    L_EC = math.sqrt(max(0, sqrt_arg))
    
    # Region-based cable length calculation
    if q1_joint_angle_degrees <= -(90 - q_switch_deg):
        # Region A: q ≤ -(90-q_switch_deg) - l_pos uses linear, l_neg uses complex
        l_pos = (SCD_min + L_BC + L_AE + 
                r_OA * ((90 - q_switch_deg) - q1_joint_angle_degrees) * (math.pi / 180) - 
                l_pos_neg_comp)
        
        if L_EC > 0 and (Ey - Py) != 0:
            l_offset = 1e-3 * 3.6762  # l_neg at -(90-q_switch_deg)
            l_neg = (l_offset + 0.1 * (L_EC + r_PD * (math.pi - math.atan2(L_EC, r_PD) - 
                    math.atan2((Ex - Px), (Ey - Py))) + l_pos_neg_comp))
        else:
            l_offset = 1e-3 * 3.6762  # l_neg at -(90-q_switch_deg)
            l_neg = l_offset + 0.1 * (SCD_min + L_BC + L_AE + l_pos_neg_comp)
            
    elif q1_joint_angle_degrees <= (90 - q_switch_deg):
        # Region C: Pure linear antagonistic
        l_pos = (SCD_min + L_BC + L_AE + 
                r_OA * ((90 - q_switch_deg) - q1_joint_angle_degrees) * (math.pi / 180) - 
                l_pos_neg_comp)
        l_neg = (SCD_min + L_BC + L_AE + 
                r_OA * ((90 - q_switch_deg) - (-q1_joint_angle_degrees)) * (math.pi / 180) + 
                l_pos_neg_comp)
                
    else:  # q1_joint_angle_degrees > (90 - q_switch_deg)
        # Region B: Mirror of Region A
        l_neg = (l_base + r_OA * ((90 - q_switch_deg) - (-q1_joint_angle_degrees)) * 
                (math.pi / 180) + l_pos_neg_comp)
        
        # Use mirrored geometry for l_pos
        Ex_mirror = OE * math.sin((-q_rad) - math.atan2(ME, OM))  # Mirror the angle
        Ey_mirror = OE * math.cos((-q_rad) - math.atan2(ME, OM))
        sqrt_arg_mirror = (Ex_mirror - Px)**2 + (Ey_mirror - Py)**2 - r_PD**2
        L_EC_mirror = math.sqrt(max(0, sqrt_arg_mirror))
        
        if L_EC_mirror > 0 and (Ey_mirror - Py) != 0:
            l_offset = 1e-3 * 3.6762  # l_pos at (90-q_switch_deg)
            l_pos = (l_offset + 0.1 * (L_EC_mirror + r_PD * (math.pi - 
                    math.atan2(L_EC_mirror, r_PD) - 
                    math.atan2((Ex_mirror - Px), (Ey_mirror - Py))) - l_pos_neg_comp))
        else:
            l_offset = 1e-3 * 3.6762  # l_pos at (90-q_switch_deg)
            l_pos = l_offset + 0.1 * (l_base - l_pos_neg_comp)
    
    # Ensure outputs are real numbers and convert to mm
    q3_pos = 24.1 + 1000 * l_pos.real if hasattr(l_pos, 'real') else 1000 * l_pos # constant + f(q1)
    q3_neg = 24.1 + 1000 * l_neg.real if hasattr(l_neg, 'real') else 1000 * l_neg # constant + f(q1)
    
    return q3_pos, q3_neg

def get_q4_pl(q2_joint_angle_degrees):
    """
    Calculate cable lengths for joints 4R and 4L based on joint 2 angle.
    
    Args:
        q2_joint_angle_degrees (float): Joint 2 angle in degrees
        
    Returns:
        tuple: (q4_pos, q4_neg) - Cable lengths in mm
    """
    # Physical constants (converted from MATLAB)
    L_AE = 1e-3 * 1.75
    L_BC = 1e-3 * 1.48
    r_PD = 1e-3 * 0.875
    r_OA = 1e-3 * 1.3
    a_lag_q = 76.53
    b_lag_horizontal = 11.08
    CD_angle = 11.08
    OM = 1e-3 * 2
    OE = 1e-3 * 2.17
    ME = 1e-3 * 0.858
    Px = -1e-3 * 1.87
    Py = 1e-3 * 1.85
    
    # Derived constants
    SCD_min = r_PD * CD_angle * math.pi / 180
    l_pos_neg_comp = 0
    l_base = SCD_min + L_BC + L_AE
    q_rad = q2_joint_angle_degrees * (math.pi / 180)
    q_switch_deg = a_lag_q - b_lag_horizontal
    
    # Calculate geometric parameters
    Ex = OE * math.sin(q_rad - math.atan2(ME, OM))
    Ey = OE * math.cos(q_rad - math.atan2(ME, OM))
    sqrt_arg = (Ex - Px)**2 + (Ey - Py)**2 - r_PD**2
    L_EC = math.sqrt(max(0, sqrt_arg))
    
    # Region-based cable length calculation
    if q2_joint_angle_degrees <= -(90 - q_switch_deg):
        # Region A: q ≤ -(90-q_switch_deg) - l_pos uses linear, l_neg uses complex
        l_pos = (SCD_min + L_BC + L_AE + 
                r_OA * ((90 - q_switch_deg) - q2_joint_angle_degrees) * (math.pi / 180) - 
                l_pos_neg_comp)
        
        if L_EC > 0 and (Ey - Py) != 0:
            l_offset = 1e-3 * 3.3992  # l_neg at -(90-q_switch_deg)
            l_neg = (l_offset + 0.1 * (L_EC + r_PD * (math.pi - math.atan2(L_EC, r_PD) - 
                    math.atan2((Ex - Px), (Ey - Py))) + l_pos_neg_comp))
        else:
            l_offset = 1e-3 * 3.3992  # l_neg at -(90-q_switch_deg)
            l_neg = l_offset + 0.1 * (SCD_min + L_BC + L_AE + l_pos_neg_comp)
            
    elif q2_joint_angle_degrees <= (90 - q_switch_deg):
        # Region C: Pure linear antagonistic
        l_pos = (SCD_min + L_BC + L_AE + 
                r_OA * ((90 - q_switch_deg) - q2_joint_angle_degrees) * (math.pi / 180) - 
                l_pos_neg_comp)
        l_neg = (SCD_min + L_BC + L_AE + 
                r_OA * ((90 - q_switch_deg) - (-q2_joint_angle_degrees)) * (math.pi / 180) + 
                l_pos_neg_comp)
                
    else:  # q1_joint_angle_degrees > (90 - q_switch_deg)
        # Region B: Mirror of Region A
        l_neg = (l_base + r_OA * ((90 - q_switch_deg) - (-q2_joint_angle_degrees)) * 
                (math.pi / 180) + l_pos_neg_comp)
        
        # Use mirrored geometry for l_pos
        Ex_mirror = OE * math.sin((-q_rad) - math.atan2(ME, OM))  # Mirror the angle
        Ey_mirror = OE * math.cos((-q_rad) - math.atan2(ME, OM))
        sqrt_arg_mirror = (Ex_mirror - Px)**2 + (Ey_mirror - Py)**2 - r_PD**2
        L_EC_mirror = math.sqrt(max(0, sqrt_arg_mirror))
        
        if L_EC_mirror > 0 and (Ey_mirror - Py) != 0:
            l_offset = 1e-3 * 3.3992  # l_pos at (90-q_switch_deg)
            l_pos = (l_offset + 0.1 * (L_EC_mirror + r_PD * (math.pi - 
                    math.atan2(L_EC_mirror, r_PD) - 
                    math.atan2((Ex_mirror - Px), (Ey_mirror - Py))) - l_pos_neg_comp))
        else:
            l_offset = 1e-3 * 3.3992  # l_pos at (90-q_switch_deg)
            l_pos = l_offset + 0.1 * (l_base - l_pos_neg_comp)
    
    # Ensure outputs are real numbers and convert to mm
    q4_pos = 28 + 1000 * l_pos.real if hasattr(l_pos, 'real') else 1000 * l_pos # constant + f(q2)
    q4_neg = 28 + 1000 * l_neg.real if hasattr(l_neg, 'real') else 1000 * l_neg # constant + f(q2)
    
    return q4_pos, q4_neg

def sanity_check():
    # Plot for Q4
    q2_angles = np.linspace(-90, 90, 181)
    q4_pos_list = []
    q4_neg_list = []
    for angle in q2_angles:
        q4_pos, q4_neg = get_q4_pl(angle)
        q4_pos_list.append(q4_pos)
        q4_neg_list.append(q4_neg)
    plt.figure()
    plt.plot(q2_angles, q4_pos_list, label='Q4 Pos')
    plt.plot(q2_angles, q4_neg_list, label='Q4 Neg')
    plt.xlabel('Q2 Joint Angle (degrees)')
    plt.ylabel('Path Length (mm)')
    plt.title('Q4 Path Lengths vs Q2 Joint Angle')
    plt.legend()
    plt.grid(True)
    plt.savefig('q4_path_lengths.png')
    plt.close()

    # Plot for Q3
    q1_angles = np.linspace(-90, 90, 181)
    q3_pos_list = []
    q3_neg_list = []
    for angle in q1_angles:
        q3_pos, q3_neg = get_q3_pl(angle)
        q3_pos_list.append(q3_pos)
        q3_neg_list.append(q3_neg)
    plt.figure()
    plt.plot(q1_angles, q3_pos_list, label='Q3 Pos')
    plt.plot(q1_angles, q3_neg_list, label='Q3 Neg')
    plt.xlabel('Q1 Joint Angle (degrees)')
    plt.ylabel('Path Length (mm)')
    plt.title('Q3 Path Lengths vs Q1 Joint Angle')
    plt.legend()
    plt.grid(True)
    plt.savefig('q3_path_lengths.png')
    plt.close()

sanity_check()
