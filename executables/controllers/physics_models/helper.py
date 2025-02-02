import numpy as np
from scipy.integrate import dblquad

class Helper:
    """Helper class containing utility functions."""

    # Gravitational acceleration constant
    g = 9.81  # m/s^2

    @staticmethod
    def C3_2d(theta):
        """
        Return a 2D rotation matrix for a given angle theta.

        Parameters:
        - theta: Rotation angle in radians.

        Returns:
        - rotation_matrix: 2x2 numpy array representing the rotation matrix.
        """
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        rotation_matrix = np.array([
            [cos_theta, -sin_theta],
            [sin_theta,  cos_theta]
        ])
        return rotation_matrix

    @staticmethod
    def S2(omega):
        """
        Return a 2D skew-symmetric matrix for a scalar omega.

        Parameters:
        - omega: Scalar value representing angular velocity.

        Returns:
        - skew_matrix: 2x2 numpy array representing the skew-symmetric matrix.
        """
        skew_matrix = np.array([
            [0,     -omega],
            [omega,    0   ]
        ])
        return skew_matrix

    @staticmethod
    def cross2d_scalar_vector(omega, vector):
        """
        Compute the cross product of a scalar omega and a 2D vector.

        Parameters:
        - omega: Scalar value (angular velocity).
        - vector: 2D numpy array.

        Returns:
        - cross_product: 2D numpy array resulting from the cross product.
        """
        cross_product = omega * np.array([-vector[1], vector[0]])
        return cross_product

    @staticmethod
    def double_gauss_quad(func, x_min, x_max, y_min, y_max):
        """
        Perform double Gaussian quadrature over specified ranges.

        Parameters:
        - func: Function to integrate, of the form func(x, y).
        - x_min, x_max: Integration limits for x.
        - y_min, y_max: Integration limits for y.

        Returns:
        - result: Result of the integration.
        """
        # Note: In dblquad, the order of variables is func(y, x)
        result, _ = dblquad(lambda y, x: func(x, y), x_min, x_max, lambda x: y_min, lambda x: y_max)
        return result