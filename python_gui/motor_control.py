import math
from constants import MotorIndex

class MotorController:
    def __init__(self):
        self.jaw_LJL_RJR_constants = {
            "L1": 10.0,  # Length constant 1 in mm
            "L2": 8.0,   # Length constant 2 in mm
            "r1": 5.0,   # Radius 1 in mm
            "r2": 5.0,   # Radius 2 in mm
            "c1x": 15.0, # Circle 1 center x in mm
            "c1y": 0.0   # Circle 1 center y in mm
        }

    def get_steps_elbow_pitch(self, degrees_ep):
        steps_ep_calc = int(degrees_ep * 100)
        motor_steps = [0] * len(MotorIndex)
        motor_steps[MotorIndex.EPU] = steps_ep_calc
        motor_steps[MotorIndex.EPD] = -steps_ep_calc
        return motor_steps

    def get_steps_elbow_yaw(self, degrees_ey):
        steps_ey_calc = int(degrees_ey * 100)
        motor_steps = [0] * len(MotorIndex)
        motor_steps[MotorIndex.EYR] = steps_ey_calc
        motor_steps[MotorIndex.EYL] = -steps_ey_calc
        return motor_steps
    
    def get_steps_wrist_pitch(self, degrees_wp, current_wp_deg):
        rad_wp = math.radians(degrees_wp)
        mm_wp = rad_wp * 1.7
        steps_wp = int(mm_wp * (200/0.3))

        motor_steps = [0] * len(MotorIndex)
        
        current_q_wrist_deg = 90 + current_wp_deg
        target_q_wrist_deg = current_q_wrist_deg + degrees_wp

        # Calculate steps for primary wrist pitch motors
        motor_steps[MotorIndex.WPD] = -steps_wp
        motor_steps[MotorIndex.WPU] = steps_wp

        # Add steps for auxiliary jaw motors
        lj_steps, rj_steps = self._calculate_auxiliary_jaw_steps(current_q_wrist_deg, target_q_wrist_deg)
        motor_steps[MotorIndex.RJL] = rj_steps
        motor_steps[MotorIndex.LJR] = lj_steps
        motor_steps[MotorIndex.LJL] = lj_steps
        motor_steps[MotorIndex.RJR] = rj_steps
        
        return motor_steps

    def get_steps_left_jaw(self, degrees_lj):
        rad_lj = math.radians(degrees_lj)
        mm_lj = rad_lj * 1.35
        steps_lj = int(mm_lj * (200/0.3))
        
        motor_steps = [0] * len(MotorIndex)
        motor_steps[MotorIndex.LJR] = steps_lj
        motor_steps[MotorIndex.LJL] = -steps_lj
        return motor_steps

    def get_steps_right_jaw(self, degrees_rj):
        rad_rj = math.radians(degrees_rj)
        mm_rj = rad_rj * 1.35
        steps_rj = int(mm_rj * (200/0.3))

        motor_steps = [0] * len(MotorIndex)
        motor_steps[MotorIndex.RJL] = -steps_rj
        motor_steps[MotorIndex.RJR] = steps_rj
        return motor_steps

    def calculate_all_motor_steps(self, ep_deg, ey_deg, wp_deg, lj_deg, rj_deg, current_wp_deg):
        total_motor_steps = [0] * len(MotorIndex)
        
        # Process each joint's contribution
        joint_steps = {
            "EP": self.get_steps_elbow_pitch(ep_deg) if ep_deg != 0 else None,
            "EY": self.get_steps_elbow_yaw(ey_deg) if ey_deg != 0 else None,
            "WP": self.get_steps_wrist_pitch(wp_deg, current_wp_deg) if wp_deg != 0 else None,
            "LJ": self.get_steps_left_jaw(lj_deg) if lj_deg != 0 else None,
            "RJ": self.get_steps_right_jaw(rj_deg) if rj_deg != 0 else None
        }

        # Combine all steps
        for steps in joint_steps.values():
            if steps is not None:
                for i in range(len(MotorIndex)):
                    total_motor_steps[i] += steps[i]

        return [int(round(s)) for s in total_motor_steps]

    def _calculate_auxiliary_jaw_steps(self, current_q_wrist_deg, target_q_wrist_deg):
        # Placeholder logic: returns (lj_steps, rj_steps) as 0 for both
        # Replace with your actual kinematic/path length calculation if needed
        return 0, 0
