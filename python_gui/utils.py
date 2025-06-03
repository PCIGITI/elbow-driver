import math

class PathLengthCalculator:
    @staticmethod
    def calculate_px_py(q2_rad, L1_const, L2_const):
        """Calculates the cable exit point (px, py) from link-2."""
        val_for_asin_arg = L2_const / L1_const
        val_for_asin_arg = max(-1.0, min(1.0, val_for_asin_arg))
        
        angle_offset = math.asin(val_for_asin_arg)
        px_val = L1_const * math.cos(q2_rad - angle_offset)
        py_val = L1_const * math.sin(q2_rad - angle_offset)
        return px_val, py_val

    @staticmethod
    def calculate_case1_pl(px_val, py_val, r1_c, c1x_c, c1y_c):
        """Calculates Path Length for Case 1: PL(q2) = l1 + s1."""
        l1_val_squared_arg = (px_val - c1x_c)**2 + (py_val - c1y_c)**2 - r1_c**2
        l1_val = math.sqrt(max(0, l1_val_squared_arg))

        alpha1_val = math.pi / 2 if r1_c == 0 and l1_val > 0 else math.atan(l1_val / r1_c) if r1_c != 0 else 0.0

        numerator_beta1 = px_val - c1x_c
        denominator_beta1 = py_val - c1y_c
        beta1_val = (math.copysign(math.pi / 2, numerator_beta1) if numerator_beta1 != 0 else 0.0) if denominator_beta1 == 0 \
                   else math.atan(numerator_beta1 / denominator_beta1)
        
        gamma1_val = (math.pi / 2) - alpha1_val - abs(beta1_val)
        s1_val = r1_c * abs(gamma1_val)
        return l1_val + s1_val

    @staticmethod
    def calculate_case2_pl(px_val, py_val, r1_c, c1x_c, c1y_c, r2_c):
        """Calculates Path Length for Case 2: PL(q2) = l2 + 2*s2 + l3 + s3."""
        c2x_internal = 0.0
        c2y_internal = 0.0

        l3_val = math.sqrt(max(0, (px_val - c2x_internal)**2 + (py_val - c2y_internal)**2 - r2_c**2))
        alpha3_val = math.pi / 2 if r2_c == 0 and l3_val > 0 else math.atan(l3_val / r2_c) if r2_c != 0 else 0.0
        beta3_val = math.atan2(py_val - c2y_internal, px_val - c2x_internal)
        gamma3_val = beta3_val - alpha3_val
        s3_val = r2_c * abs(gamma3_val)

        dist_sq_c1_c2 = (c1x_c - c2x_internal)**2 + (c1y_c - c2y_internal)**2
        l2_val = math.sqrt(max(0, dist_sq_c1_c2 - (r1_c + r2_c)**2))

        if dist_sq_c1_c2 > 0:
            arg_acos_alpha2 = (r1_c + r2_c) / math.sqrt(dist_sq_c1_c2)
            arg_acos_alpha2 = max(-1.0, min(1.0, arg_acos_alpha2))
            alpha2_val = math.acos(arg_acos_alpha2)
        else:
            alpha2_val = 0

        den_beta2_val = abs(c1y_c - c2y_internal)
        beta2_val = math.pi / 2 if den_beta2_val == 0 and abs(c1x_c - c2x_internal) > 0 \
                   else math.atan(abs(c1x_c - c2x_internal) / den_beta2_val) if den_beta2_val != 0 else 0.0
        
        gamma2_val = (math.pi / 2) - alpha2_val - beta2_val
        s2_component = r2_c * abs(gamma2_val)
        
        return l2_val + (2 * s2_component) + l3_val + s3_val
