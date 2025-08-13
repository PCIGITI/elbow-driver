import math

def l_pos_neg_py(j_num, q_rad):
    """
    Calculates the lengths of the two antagonistic cables for a given joint based on its angle.
    This is a direct Python translation of the l_pos_neg function from the MATLAB script,
    which models the detailed, non-linear geometry of the cable paths.

    Args:
        j_num (int): The joint number being analyzed (3 for the Q1->Q3 coupling).
        q_rad (float): The absolute angle of the primary joint (Q1) in radians.

    Returns:
        tuple[float, float]: A tuple containing the lengths (l_pos, l_neg) in meters.
    """
    # Define physical constants from the MATLAB script based on the joint being analyzed.
    # All length units are converted to meters to match the script.
    if j_num == 3:  # For Q1 -> Q3 coupling (Elbow Pitch -> Wrist)
        L_AE = 0.00191
        L_BC = 0.00109
        r_PD = 0.0015
        r_OA = 0.00125
        a_lag_q = 66.92  # degrees
        b_lag_horizontal = 25.83  # degrees
        OM = 0.00225
        OE = 0.00229
        ME = 0.0004
        Px = -0.002
        Py = 0.00218
        Px_prime = -0.002
        Py_prime = -0.00218
    else:
        # Constants for other couplings (like Q2->Q4) can be added here later.
        raise ValueError("j_num must be 3 for the Q1->Q3 coupling model.")

    q_deg = math.degrees(q_rad)
    q_switch_deg = a_lag_q - b_lag_horizontal

    # Calculate cable exit points from the guide pulley (E and E')
    angle_offset = math.atan2(ME, OM) # Use atan2 for quadrant-correct angle
    Ex = OE * math.sin(q_rad - angle_offset)
    Ey = OE * math.cos(q_rad - angle_offset)
    Ex_prime = Ex
    Ey_prime = -Ey

    # Avoid math domain errors by ensuring arguments to sqrt are non-negative
    sqrt_arg1 = max(0, (Ex - Px)**2 + (Ey - Py)**2 - r_PD**2)
    sqrt_arg2 = max(0, (Ex_prime - Px_prime)**2 + (Ey_prime - Py_prime)**2 - r_PD**2)
    L_EC = math.sqrt(sqrt_arg1)
    L_EC_prime = math.sqrt(sqrt_arg2)

    CD_angle_rad = math.radians(b_lag_horizontal)
    SCD_min = r_PD * CD_angle_rad

    # Clamp q_deg to physical limits [-90, 90] to be safe
    q_deg = max(-90.0, min(90.0, q_deg))

    # This logic from the MATLAB script determines which geometric case applies based on the joint angle
    if q_deg <= -90 + q_switch_deg:
        l_neg = L_EC + r_PD * (math.pi - math.atan2(L_EC, r_PD) - math.atan2((Ex - Px), (Ey - Py)))
        l_pos = SCD_min + L_BC + L_AE + r_OA * math.radians(90 - q_switch_deg - q_deg)
    elif q_deg <= 90 - q_switch_deg:
        l_neg = SCD_min + L_BC + L_AE + r_OA * math.radians(q_deg - (a_lag_q - b_lag_horizontal))
        l_pos = SCD_min + L_BC + L_AE + r_OA * math.radians(90 - q_switch_deg - q_deg)
    else:  # q_deg > 90 - q_switch_deg
        l_neg = SCD_min + L_BC + L_AE + r_OA * math.radians(q_deg - (a_lag_q - b_lag_horizontal))
        l_pos = L_EC_prime + r_PD * (math.pi - math.atan2(L_EC_prime, r_PD) - math.atan2((Ex_prime - Px_prime), (Ey_prime - Py_prime)))

    return l_pos, l_neg
