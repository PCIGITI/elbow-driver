import math

class Joint:
    """
    Represents a single robot joint, encapsulating its state, kinematic relationships,
    and the logic for how its movement affects other parts of the system.
    
    All calculations are done in radians and rad/s.
    """
    def __init__(self, name, primary_motor_indices, initial_angle_rad=math.pi/2, motor_ratio=1.0):
        """
        Initializes a Joint object.

        Args:
            name (str): The name of the joint (e.g., "EP", "ROLL").
            primary_motor_indices (list[int]): A list of motor indices from config.MotorIndex
                                               that directly drive this joint. For a differential
                                               pair, this would be two indices.
            initial_angle_rad (float): The starting angle of the joint in radians.
            motor_ratio (float): The ratio to convert joint angular velocity (rad/s) to
                                 primary motor velocity. For a differential pair, this is
                                 typically 1.0 for the positive motor and -1.0 for the negative.
        """
        self.name = name
        self.angle_rad = initial_angle_rad
        self.primary_motor_indices = primary_motor_indices
        self.motor_ratio = motor_ratio
        
        # This list will store functions that calculate compensation for other motors.
        self.compensation_rules = []

    def add_compensation(self, affected_motor_indices, length_func, derivative_func=None):
        """
        Adds a compensation rule for how this joint's movement affects other motors.

        Args:
            affected_motor_indices (list[int]): The motor indices that are affected.
            length_func (callable): A function L(theta) that takes this joint's angle (rad)
                                    and returns a required length or position for the
                                    affected motors.
            derivative_func (callable, optional): An analytical derivative dL/d(theta) of the
                                                  length function. If not provided, a numerical
                                                  derivative will be used.
        """
        if derivative_func is None:
            # If no analytical derivative is provided, create a numerical one.
            delta_theta = 0.0001  # A small angle in radians for numerical differentiation
            derivative = lambda theta: (length_func(theta + delta_theta) - length_func(theta)) / delta_theta
        else:
            derivative = derivative_func
            
        self.compensation_rules.append({
            "affected_motors": affected_motor_indices,
            "derivative": derivative
        })

    def calculate_velocities(self, joint_angular_velocity_rad_s):
        """
        Calculates all motor velocities resulting from this joint's movement.

        Args:
            joint_angular_velocity_rad_s (float): The desired angular velocity for this joint in rad/s.

        Returns:
            dict: A dictionary mapping motor_index to its calculated velocity.
        """
        # This dictionary will hold all resulting velocities {motor_index: velocity}
        velocities = {}

        # 1. Calculate primary motor velocities
        # Assumes the first primary motor moves positively, the second negatively (for a differential pair)
        if len(self.primary_motor_indices) > 0:
            velocities[self.primary_motor_indices[0]] = joint_angular_velocity_rad_s * self.motor_ratio
        if len(self.primary_motor_indices) > 1:
            velocities[self.primary_motor_indices[1]] = joint_angular_velocity_rad_s * -self.motor_ratio

        # 2. Calculate compensation velocities for all other affected motors
        for rule in self.compensation_rules:
            # The core relationship: dL/dt = (dL/d_theta) * (d_theta/dt)
            # We calculate dL/d_theta at the *current* joint angle.
            geometric_factor = rule["derivative"](self.angle_rad)
            compensation_velocity = geometric_factor * joint_angular_velocity_rad_s
            
            for motor_idx in rule["affected_motors"]:
                # Use .get(key, 0) to safely add to existing velocity if another
                # joint also compensates for this motor.
                velocities[motor_idx] = velocities.get(motor_idx, 0) + compensation_velocity
                
        return velocities

    def update_angle(self, delta_angle_rad):
        """Updates the joint's internal angle."""
        self.angle_rad += delta_angle_rad
    
    def set_angle(self, angle_rad):
        """Sets the joint's internal angle to an absolute value."""
        self.angle_rad = angle_rad

