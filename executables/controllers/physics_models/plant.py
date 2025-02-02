import numpy as np
from scipy.integrate import dblquad
from helper import Helper

class PlanarSystem:
    """Class representing the planar mechanical system."""

    def __init__(self, parameters, object_properties, system_state):
        """
        Initialize the PlanarSystem with parameters, object properties, and system state.

        Parameters:
        - parameters: Dictionary containing system parameters (e.g., control inputs).
        - object_properties: Dictionary containing object properties (e.g., mass, dimensions).
        - system_state: Dictionary containing system state parameters (e.g., friction coefficient).
        """
        # Store parameters
        self.p = parameters            # Parameters (e.g., control inputs)
        self.o = object_properties     # Object properties (e.g., mass, dimensions)
        self.s = system_state          # System state parameters (e.g., friction coefficient)

        # Initialize variable dimensions
        self.initialize_variables()

        # Compute maximum allowable force and moment based on friction and object properties
        self.f_max = self.s['nu'] * self.o['m'] * Helper.g
        self.m_max = self.compute_m_max()
        self.c = self.m_max / self.f_max  # Ratio of maximum moment to maximum force

        # Build limit surface representation for friction constraints
        self.build_limit_surface()

    def initialize_variables(self):
        """Initialize variable dimensions based on parameters."""
        # Number of continuous states (e.g., x, y, theta, ry)
        self.num_xcStates = 4
        # Number of control inputs
        self.num_ucStates = len(self.p['uc'])
        # Number of system states
        self.num_xsStates = 3 + len(self.p['xp'])
        # Number of system inputs
        self.num_usStates = len(self.p['xp'])

    def compute_m_max(self):
        """
        Compute the maximum frictional moment (m_max) by integrating over the object's area.

        Returns:
        - m_max: Maximum frictional moment.
        """
        # Define the integrand function for the moment due to friction
        def integrand(x, y):
            # Friction force per unit area
            friction_force_density = (self.s['nu'] * self.o['m'] * Helper.g) / self.o['A']
            # Position vector from the center of mass
            r = np.array([x, y])
            # Moment arm is the dis tance from the center (norm of position vector)
            moment_arm = np.linalg.norm(r)
            # Integrate the friction force over the moment arm
            return friction_force_density * moment_arm

        # Integration limits (object dimensions)
        x_min, x_max = -self.o['a'] / 2, self.o['a'] / 2
        y_min, y_max = -self.o['b'] / 2, self.o['b'] / 2

        # Perform double integration over the object's area to compute m_max
        m_max = Helper.double_gauss_quad(integrand, x_min, x_max, y_min, y_max)
        return m_max

    def build_limit_surface(self):
        """
        Build the limit surface representation for friction constraints.
        """
        # Since H is quadratic in fx, fy, and m, the Hessian is constant
        self.A_ls = np.diag([
            2 / self.f_max**2,  # For fx
            2 / self.f_max**2,  # For fy
            2 / self.m_max**2   # For m
        ])

    def coordinate_transform_SC(self, xs):
        """
        Transform system states xs to continuous states xc.

        Parameters:
        - xs: System states (numpy array), e.g., [x_i_b_i, theta, x_i_p_i]

        Returns:
        - xc: Continuous states (numpy array), e.g., [x_i_b_i, theta, ry]
        """
        # Extract variables from system states
        position_inertial = xs[:2]          # Object position in inertial frame
        orientation = xs[2]                 # Object orientation
        pusher_position_inertial = xs[3:5]  # Pusher position in inertial frame

        # Compute the rotation matrix from inertial to body frame
        rotation_matrix = Helper.C3_2d(orientation)

        # Compute object position in body frame (should be zero vector)
        position_body = np.dot(rotation_matrix, position_inertial)

        # Compute pusher position in body frame
        pusher_position_body = np.dot(rotation_matrix, pusher_position_inertial)

        # Compute relative position in body frame (pusher relative to object)
        relative_position_body = pusher_position_body - position_body

        # Extract ry (y-coordinate of pusher relative to object in body frame)
        ry = relative_position_body[1]

        # Build continuous state vector xc
        xc = np.concatenate([position_inertial, [orientation, ry]])
        return xc

    def coordinate_transform_CS(self, xc):
        """
        Transform continuous states xc to system states xs.

        Parameters:
        - xc: Continuous states (numpy array), e.g., [x_i_b_i, theta, ry]

        Returns:
        - xs: System states (numpy array), e.g., [x_i_b_i, theta, x_i_p_i]
        """
        # Extract variables from continuous states
        position_inertial = xc[:2]  # Object position in inertial frame
        orientation = xc[2]         # Object orientation
        ry = xc[3]                  # ry: y-coordinate of pusher in body frame

        # Compute the rotation matrix from body to inertial frame
        rotation_matrix = Helper.C3_2d(orientation)

        # Pusher position in body frame (assuming pusher is at x = -a/2 in body frame)
        pusher_position_body = np.array([-self.o['a'] / 2, ry])

        # Convert pusher position to inertial frame
        pusher_position_inertial = position_inertial + np.dot(rotation_matrix.T, pusher_position_body)

        # Build system state vector xs
        xs = np.concatenate([position_inertial, [orientation], pusher_position_inertial])

        # If pusher state includes orientation, append it (assuming same as object orientation)
        if len(self.p['xp']) == 3:
            xs = np.concatenate([xs, [orientation]])

        return xs

    def force_simulator(self, xc, uc):
        """
        Simulate the system dynamics given the continuous states xc and control inputs uc.

        Parameters:
        - xc: Continuous states (numpy array), e.g., [x_i_b_i, theta, ry]
        - uc: Control inputs (numpy array), e.g., [fn1, ft1, ry_dot]

        Returns:
        - dxs: Derivative of the system states (numpy array)
        """
        # Extract variables from continuous state
        position_inertial = xc[:2]  # Object position in inertial frame
        orientation = xc[2]         # Object orientation
        ry = xc[3]                  # y-coordinate of pusher in body frame

        # Compute the rotation matrix from inertial to body frame
        rotation_matrix = Helper.C3_2d(orientation)

        # Compute the twist (velocities) of the object in inertial frame
        twist_object_inertial = self.compute_twist_object(xc, uc)

        # Compute the pusher velocity in inertial frame
        pusher_velocity_inertial = self.force_to_velocity(xc, uc)

        # Combine the object twist and pusher velocity to form state derivatives
        dxs = np.concatenate([twist_object_inertial, pusher_velocity_inertial])

        return dxs

    def compute_twist_object(self, xc, uc):
        """
        Compute the twist (velocities) of the object given states and controls.

        Parameters:
        - xc: Continuous states (numpy array)
        - uc: Control inputs (numpy array)

        Returns:
        - twist_object_inertial: Object twist in inertial frame (numpy array)
        """
        # Placeholder for the actual computation using dynamics equations
        # For this example, we will assume a simple proportional relationship
        # between control inputs and object twist
        fn, ft, ry_dot = uc  # Extract control inputs

        # Compute force vector in body frame
        force_body = np.array([fn, ft])

        # Compute the rotation matrix from body to inertial frame
        orientation = xc[2]
        rotation_matrix = Helper.C3_2d(orientation)

        # Convert force to inertial frame
        force_inertial = np.dot(rotation_matrix.T, force_body)

        # Assume mass and inertia properties
        mass = self.o['m']
        inertia = self.o['I']

        # Compute linear acceleration (Newton's second law)
        linear_acceleration = force_inertial / mass

        # Compute angular acceleration (assuming torque is ft * lever arm)
        torque = ft * self.o['a'] / 2  # Lever arm is half the object's width
        angular_acceleration = torque / inertia

        # Combine linear and angular velocities into twist
        twist_object_inertial = np.concatenate([linear_acceleration, [angular_acceleration]])

        return twist_object_inertial

    def force_to_velocity(self, xc, uc):
        """
        Convert reaction forces to the associated pusher velocity.

        Parameters:
        - xc: Continuous states (numpy array)
        - uc: Control inputs (numpy array)

        Returns:
        - pusher_velocity_inertial: Pusher velocity in inertial frame (numpy array)
        """
        # Extract variables
        orientation = xc[2]
        ry = xc[3]
        rx = -self.o['a'] / 2  # x-coordinate of pusher in body frame

        # Pusher position in body frame
        pusher_position_body = np.array([rx, ry])

        # Compute the rotation matrix from body to inertial frame
        rotation_matrix = Helper.C3_2d(orientation)

        # Compute object twist
        twist_object_inertial = self.compute_twist_object(xc, uc)
        angular_velocity = twist_object_inertial[2]

        # Compute pusher velocity relative to object in body frame
        pusher_velocity_body = Helper.cross2d_scalar_vector(angular_velocity, pusher_position_body)

        # Convert pusher velocity to inertial frame
        pusher_velocity_inertial = np.dot(rotation_matrix.T, pusher_velocity_body)

        # Add any additional velocity due to ry_dot (from control inputs)
        ry_dot = uc[2]  # Extract ry_dot from control inputs
        pusher_velocity_inertial += np.array([0, ry_dot])

        return pusher_velocity_inertial

    # Additional methods can be added here as needed for full functionality