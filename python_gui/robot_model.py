# This script defines the physical model of the robot, completely independent of
# the control UI and motor hardware. It takes commanded joint angular velocities
# (rad/s) as input and calculates the final, compensated linear velocities (e.g., mm/s)
# required for each joint's cable(s). This output is then passed to a separate
# "Driver Map" layer, which translates these physical velocities into hardware-specific
# motor commands.

import math

# --- 1. Define the Reusable Mathematical Functions ---

def _kinematic_derivative_func(theta_ind, params):
    """
    The ANALYTICAL DERIVATIVE (d_theta_dep / d_theta_ind) for kinematic couplings.
    For a linear relationship where induced_angle = r2 * independent_angle,
    the derivative is simply the constant r2. It does not depend on joint angles.
    """
    return params['r2']

def _tensile_spiral_math_func(theta_ind):
    """
    The function for 'spiral' type tensile couplings.
    Calculates the REQUIRED cable length change for a dependent joint.
    """
    # TODO: Implement spiral tensile math, which only depends on theta_ind.
    return 0.0

def _tensile_circle_math_func(theta_ind):
    """
    The function for 'circle' type tensile couplings.
    Calculates the REQUIRED cable length change for a dependent joint.
    """
    # TODO: Implement circle tensile math, which only depends on theta_ind.
    return 0.0


# --- COUPLING EFFECT CLASSES ---

class CouplingEffectBase:
    """An abstract base class for all coupling effects."""
    def __init__(self, dependent_joint):
        self.dependent_joint = dependent_joint

    def calculate_compensation(self, independent_joint_angle, independent_joint_velocity):
        """Calculates the resulting compensatory velocity."""
        raise NotImplementedError("Subclasses must implement this method.")

class KinematicCouplingEffect(CouplingEffectBase):
    """
    Represents a kinematic coupling, which uses a pre-calculated,
    analytical derivative function for maximum accuracy and performance.
    """
    def __init__(self, dependent_joint, derivative_func, geo_params):
        super().__init__(dependent_joint)
        if not callable(derivative_func):
            raise TypeError("derivative_func must be a callable function.")
        self.derivative_func = derivative_func
        self.geo_params = geo_params
        self.type = 'kinematic'

    def calculate_compensation(self, independent_joint_angle, independent_joint_velocity):
        # Directly use the provided analytical derivative function.
        geometric_factor = self.derivative_func(independent_joint_angle, self.geo_params)
        # Required vel = - (d_theta_dep / d_theta_ind) * vel_ind
        return -geometric_factor * independent_joint_velocity

class TensileCouplingEffect(CouplingEffectBase):
    """
    Represents a tensile coupling. It calculates the derivative numerically
    from a base math function selected by its type.
    """
    def __init__(self, dependent_joint, type):
        super().__init__(dependent_joint)
        if type not in ['spiral', 'circle']:
            raise ValueError("type must be 'spiral' or 'circle'")
        self.type = 'tensile'
        self.coupling_type = type
        
        if self.coupling_type == 'spiral':
            self.coupling_func = _tensile_spiral_math_func
        else: # 'circle'
            self.coupling_func = _tensile_circle_math_func

    def _get_numerical_derivative(self, theta_ind):
        """Numerically calculates the derivative of the assigned coupling function."""
        delta_theta = 0.0001
        val1 = self.coupling_func(theta_ind)
        val2 = self.coupling_func(theta_ind + delta_theta)
        return (val2 - val1) / delta_theta

    def calculate_compensation(self, independent_joint_angle, independent_joint_velocity):
        # Use the internal numerical derivative method.
        geometric_factor = self._get_numerical_derivative(independent_joint_angle)
        # Required cable vel = (dL/d_theta_ind) * vel_ind
        return geometric_factor * independent_joint_velocity

# --- JOINT CLASS ---
class Joint:
    """
    Represents a single, abstract joint in the robot. It holds its own state
    and a dictionary of the effects it has on other joints.
    """
    def __init__(self, name, effective_radius):
        self.name = name
        self.effective_radius = effective_radius
        self.angle_rad = 0.0
        self.coupling_effects = {}

    def add_effect(self, effect: CouplingEffectBase):
        """Adds a pre-configured coupling effect to this joint."""
        dependent_name = effect.dependent_joint.name
        self.coupling_effects[dependent_name] = effect

# --- ROBOT MODEL ---
class RobotModel:
    """
    Represents the complete robot by defining its joints and configuring the
    explicit couplings that belong to each joint.
    """
    def __init__(self):
        # --- 1. Define Geometric Constants for Kinematic Couplings ---
        kinematic_params_q1_q3 = {'r2': 1.5}
        kinematic_params_q2_q4 = {'r2': 1.3}

        # --- 2. Initialize Joints ---
        self.joints = {
            'q0': Joint('q0', effective_radius=1.5),
            'q1': Joint('q1', effective_radius=1.5),
            'q2': Joint('q2', effective_radius=1.3),
            'q3': Joint('q3', effective_radius=1.7),
            'q4': Joint('q4', effective_radius=1.35)
        }

        # --- 3. Create and Assign Coupling Effect Objects ---
        self.joints['q1'].add_effect(KinematicCouplingEffect(self.joints['q3'], _kinematic_derivative_func, kinematic_params_q1_q3))
        self.joints['q1'].add_effect(TensileCouplingEffect(self.joints['q4'], 'spiral'))

        self.joints['q2'].add_effect(KinematicCouplingEffect(self.joints['q4'], _kinematic_derivative_func, kinematic_params_q2_q4))
        self.joints['q2'].add_effect(TensileCouplingEffect(self.joints['q3'], 'spiral'))
        
        self.joints['q3'].add_effect(TensileCouplingEffect(self.joints['q4'], 'circle'))

    def get_final_cable_velocities(self, commanded_joint_velocities):
        """
        Takes a dictionary of commanded joint angular velocities (rad/s) and
        returns a single dictionary of the final required linear velocity
        (e.g., mm/s) for each joint's cable(s).
        """
        # --- Pass 1: Calculate Kinematic and Tensile Compensations ---
        final_joint_velocities = commanded_joint_velocities.copy()
        compensatory_cable_velocities = {name: 0.0 for name in self.joints}

        for ind_name, ind_vel in commanded_joint_velocities.items():
            if ind_vel == 0: continue
            independent_joint = self.joints[ind_name]
            
            for dep_name, effect in independent_joint.coupling_effects.items():
                compensatory_vel = effect.calculate_compensation(
                    independent_joint.angle_rad,
                    ind_vel
                )
                
                if effect.type == 'kinematic':
                    final_joint_velocities[dep_name] = final_joint_velocities.get(dep_name, 0) + compensatory_vel
                elif effect.type == 'tensile':
                    compensatory_cable_velocities[dep_name] += compensatory_vel

        # --- Pass 2: Calculate Final Cable Velocities ---
        # This combines the primary motion (from joint rotation) with the
        # tensile compensation motion into a single value per joint.
        final_cable_velocities = {}
        for name, joint in self.joints.items():
            # Convert the joint's final angular velocity to a linear cable velocity.
            primary_cable_velocity = final_joint_velocities.get(name, 0) * joint.effective_radius
            
            # Add the tensile compensation velocity.
            total_cable_velocity = primary_cable_velocity + compensatory_cable_velocities.get(name, 0)
            
            final_cable_velocities[name] = total_cable_velocity
            
        return final_cable_velocities

    def update_angles(self, final_joint_velocities, time_delta_s):
        """
        Updates the internal angle of each joint based on the final calculated
        velocities and the elapsed time.
        """
        for name, vel in final_joint_velocities.items():
            if name in self.joints:
                self.joints[name].angle_rad += vel * time_delta_s

    def reset_state(self):
        """Resets all joint angles to 0."""
        for joint in self.joints.values():
            joint.angle_rad = 0.0
