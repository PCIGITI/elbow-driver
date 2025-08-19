##WRIST MATH

import math
import numpy as np # If you keep numpy for specific calculations
import config as cf


#constant for the wrist pitch cable length change geometry
##centered around the axis as 0,0


L1_c = 1.457  # Length of link-1 in mm
L2_c = 0.55   # Length of link-2 in mm
r1_c = 0.89   # Radius of circle C1 in mm
c1x_c = 1.6   # x-coordinate of center of circle C1 in mm
c1y_c = -1.5  # y-coordinate of center of circle C1 in mm
r2_c = 1.0   # Radius of circle C2 in mm (assumed to be 0.0 for simplification)
dir_offset = cf.Q3_DR_COMP


WP_EFFECTIVE_RADIUS_MM = 1.7

def _calculate_px_py(q2_rad):
        """
        Calculates the cable exit point (px, py) from link-2. (Equation 2)
        q2_rad: Angle q2 in radians.
        L1_const, L2_const: Length constants in mm
        """
        # Handle potential domain error for asin if L2_const > L1_const
        val_for_asin_arg = L2_c / L1_c
        if not (-1.0 <= val_for_asin_arg <= 1.0):
            # This case should ideally not happen with valid geometry.
            # Log an error or return a specific failure indicator.
            # For now, let's clamp it, though this indicates a geometry issue.
            # self.log_message(f"Warning: asin argument {val_for_asin_arg} out of range. Clamping.")
            val_for_asin_arg = max(-1.0, min(1.0, val_for_asin_arg))
        
        angle_offset = math.asin(val_for_asin_arg)
        px_val = L1_c * math.cos(q2_rad - angle_offset)
        py_val = L1_c * math.sin(q2_rad - angle_offset)
        return px_val, py_val

def _calculate_case1_pl(px_val, py_val):
        """
        Calculates Path Length for Case 1: PL(q2) = l1 + s1.
        """
        l1_val_squared_arg = (px_val - c1x_c)**2 + (py_val - c1y_c)**2 - r1_c**2
        if l1_val_squared_arg < 0:
            # This can happen if P is inside C1 or r1 is too large for the geometry
            # self.log_message(f"Warning: Negative sqrt argument for l1 in Case 1: {l1_val_squared_arg}")
            l1_val_squared_arg = 0 # Avoid math domain error, but indicates issue
        l1_val = math.sqrt(l1_val_squared_arg)

        # atan(l1/r1)
        if r1_c == 0: # Avoid division by zero
            alpha1_val = math.pi / 2 if l1_val > 0 else 0.0
        else:
            alpha1_val = math.atan(l1_val / r1_c)

        # atan((px-c1x)/(py-c1y))
        numerator_beta1 = px_val - c1x_c
        denominator_beta1 = py_val - c1y_c
        if denominator_beta1 == 0: # Avoid division by zero for beta1
             beta1_val = math.copysign(math.pi / 2, numerator_beta1) if numerator_beta1 != 0 else 0.0
        else:
            beta1_val = math.atan(numerator_beta1 / denominator_beta1)
        
        gamma1_val = (math.pi / 2) - alpha1_val - abs(beta1_val)
        s1_val = r1_c * abs(gamma1_val)
        return l1_val + s1_val

def _calculate_case2_pl(px_val, py_val): # Removed c2x_c, c2y_c
        """
        Calculates Path Length for Case 2: PL(q2) = l2 + 2*s2 + l3 + s3.
        c2x and c2y are assumed to be 0.0.
        """
        c2x_internal = 0.0
        c2y_internal = 0.0

        l3_val_squared_arg = (px_val - c2x_internal)**2 + (py_val - c2y_internal)**2 - r2_c**2
        if l3_val_squared_arg < 0:
            # self.log_message(f"Warning: Negative sqrt argument for l3 in Case 2: {l3_val_squared_arg}")
            l3_val_squared_arg = 0
        l3_val = math.sqrt(l3_val_squared_arg)

        if r2_c == 0:
            alpha3_val = math.pi / 2 if l3_val > 0 else 0.0
        else:
            alpha3_val = math.atan(l3_val / r2_c)
        
        beta3_val = math.atan2(py_val - c2y_internal, px_val - c2x_internal) # Use (py - 0), (px - 0)

        # User's gamma3 logic from provided code
        # if beta3_val < 0: 
        #     gamma3_val = (math.pi / 2) - alpha3_val + beta3_val
        # else: # beta3_val >= 0
        #     gamma3_val = (-math.pi / 2) - alpha3_val + beta3_val
        gamma3_val = beta3_val - alpha3_val # Using the simplified one from user's code for now

        s3_val = r2_c * abs(gamma3_val)

        dist_sq_c1_c2 = (c1x_c - c2x_internal)**2 + (c1y_c - c2y_internal)**2
        l2_squared_arg = dist_sq_c1_c2 - (r1_c + r2_c)**2
        if l2_squared_arg < 0:
            # self.log_message(f"Warning: Negative sqrt argument for l2 in Case 2: {l2_squared_arg}")
            l2_squared_arg = 0
        l2_val = math.sqrt(l2_squared_arg)

        denom_alpha2_arg = dist_sq_c1_c2
        if denom_alpha2_arg <= 0 : # or (r1_c + r2_c)**2 > dist_sq_c1_c2
             # This implies C1 and C2 are overlapping or r1+r2 is too large
             # self.log_message(f"Warning: Invalid geometry for alpha2 calculation in Case 2.")
             alpha2_val = 0 # Default, or handle error
        else:
            denom_alpha2 = math.sqrt(denom_alpha2_arg)
            arg_acos_alpha2 = (r1_c + r2_c) / denom_alpha2
            if not (-1.0 <= arg_acos_alpha2 <= 1.0):
                # self.log_message(f"Warning: acos argument {arg_acos_alpha2} out of range for alpha2. Clamping.")
                arg_acos_alpha2 = max(-1.0, min(1.0, arg_acos_alpha2))
            alpha2_val = math.acos(arg_acos_alpha2)

        den_beta2_val = abs(c1y_c - c2y_internal)
        if den_beta2_val == 0:
            beta2_val = math.pi / 2 if abs(c1x_c - c2x_internal) > 0 else 0.0
        else:
            beta2_val = math.atan(abs(c1x_c - c2x_internal) / den_beta2_val)
        
        gamma2_val = (math.pi / 2) - alpha2_val - beta2_val
        s2_component_from_eq17 = r2_c * abs(gamma2_val)
        
        return l2_val + (2 * s2_component_from_eq17) + l3_val + s3_val

def _calculate_pl_value(q2_deg_val):
        """
        Calculates the path length PL for LJL/RJR cables for a given q2.
        """
        q2_rad_val = math.radians(q2_deg_val)
        px_val, py_val = _calculate_px_py(q2_rad_val)
        
        if (c1x_c - px_val) < r1_c: # Condition from user's code
            pl_res = _calculate_case1_pl(px_val, py_val)
        else:
            pl_res = _calculate_case2_pl(px_val, py_val) # Pass only r2_c
            
        return pl_res

def get_steps(curr_theta, delta_theta, latest_dir):
        motor_steps = [0] * len(cf.MotorIndex)
        if delta_theta == 0:
            return motor_steps, latest_dir
        ##primary cables:
        mm_wp = math.radians(delta_theta)*1.7
        steps_wp = int(mm_wp*cf.STEPS_TO_MM_LS)
        
        if (latest_dir == 0 or latest_dir*delta_theta < 0 and cf.DIR_COMP):
            ###latest_dir and delta_theta are not the same direction/sign
            ###have to add compensation and swap direction
            comp = math.copysign(dir_offset, steps_wp)
            if(latest_dir == 0):
                 comp = comp/2
            steps_wp += comp
            latest_dir = delta_theta
            print(f"Added {comp} steps due to a change in direction! new latest dir: {latest_dir}")
        else:

            print(f"latest_dir didnt change or dir_comp is off: latest_dir = {latest_dir} delta theta = {delta_theta} and dr comp = {cf.DIR_COMP}")
            print("oopsies")

        ##auxiliary cables: 
        target_theta = curr_theta + delta_theta

        # Calculate for Left Jaw side (shortest when wrist is all the way up, theta = 0)
        try:
            curr_lj = _calculate_pl_value(curr_theta)
            target_lj = _calculate_pl_value(target_theta)

            curr_rj = _calculate_pl_value(180.0-curr_theta)
            target_rj = _calculate_pl_value(180.0-target_theta)           
        except Exception as e:
            print(f"Error in PL calculation: {e}")
            curr_lj = target_lj = curr_rj = target_rj = 0.0 

        delta_lj = target_lj - curr_lj
        delta_rj = target_rj - curr_rj

        print(f"Wrist pitch delta: {delta_theta}, current_abs_wp: {curr_theta}, target_abs_wp: {target_theta}")
        print(f"L_current_LJL_LJR: {curr_lj:.4f}, L_target_LJL_LJR: {target_lj:.4f}, delta_L: {delta_lj:.4f}")
        print(f"L_current_RJ: {curr_rj:.4f}, L_target_RJ: {target_rj:.4f}, delta_L: {delta_rj:.4f}")

        #positive steps = cable LENGTHENING (dynamixel motors aug 14 2025)
        
        ##positive steps_wp moves Q3 DOWN
        ##therefore, we want to lengthen (positive) LJ 
        ##and shorten (negative) RJ


        steps_lj = -int(delta_lj*cf.STEPS_TO_MM_LS)
        steps_rj = -int(delta_rj*cf.STEPS_TO_MM_LS)
        motor_steps[cf.MotorIndex.WPD] = -steps_wp
        motor_steps[cf.MotorIndex.WPU] = steps_wp

        
        motor_steps[cf.MotorIndex.RJL] = steps_rj
        motor_steps[cf.MotorIndex.LJR] = steps_lj
        motor_steps[cf.MotorIndex.LJL] = steps_lj
        motor_steps[cf.MotorIndex.RJR] = steps_rj
        return motor_steps, latest_dir
