import math
import config as cf

ROLL_RADIUS = 3 # mm, radius of the roll capstan
CAPSTAN_RADIUS = 11 # mm, radius of the capstan on the motor
MOTOR_STEPS_PER_REVOLUTION = 200 # steps per revolution (of the capstan/motor)

def get_steps(curr_theta, delta_theta): # curr_theta is often unused for simple roll but kept for API consistency
    motor_steps = [0] * len(cf.MotorIndex)

    if delta_theta == 0:
        return motor_steps

    # Condensed math for calculating steps:
    # Line 1: Calculate necessary capstan revolutions
    capstan_revolutions_needed = (math.radians(delta_theta) * ROLL_RADIUS) / (2 * math.pi * CAPSTAN_RADIUS)
    # Line 2: Convert revolutions to integer steps
    calculated_steps = int(round(capstan_revolutions_needed * MOTOR_STEPS_PER_REVOLUTION))
 
    motor_steps[cf.MotorIndex.ROLL] = calculated_steps # ROLL is an IntEnum member
    
    return motor_steps
