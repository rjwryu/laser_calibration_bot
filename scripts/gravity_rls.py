#!/usr/bin/env python
# Author: Su Jing Long, Brian
desc = """\
Real-time gravity compensation algorithm, using recursive least squares.
"""


from dataclasses import dataclass

import numpy as np


@dataclass
class GravityModelParams:
    k: float        # torque constant
    b: float        # friction coefficient
    j: float        # moment of inertia
    mgr: float      # mass * gravitational constant * distance of CoM
    alpha: float    # angle zero offset


class GravityRLS:
    def __init__(self, params: GravityModelParams, lmbda=0.995, delta=100.0):
        self.k = params.k
        self.lmbda = lmbda  # Forgetting factor (0.9 to 0.999)
        self.P = np.eye(1) * delta     # Initial uncertainty matrix
        self.theta = np.array([         # Coefficients to learn
            [params.mgr],
        ])
        self.theta_bounds = np.array([  # Clamp coefficients
            [0, np.inf],
        ])

    def get_params(self) -> GravityModelParams:
        """ Obtain model parameters. """
        return GravityModelParams(k=self.k, b=np.nan, j=np.nan, mgr=self.theta[0, 0], alpha=np.nan)

    def update(self, pos: float, cur: float) -> None:
        """
        Update the recursive solver with new data.

        Parameters:
        - pos (float): Absolute angular position in degrees
        - cur (float): Feedback current in amperes

        Returns:
        - None
        """

        # 1. Create the regressor vector for the current angle
        pos = (pos % 360) * np.pi / 180     # Convert to radians
        phi = np.array([
            [np.cos(pos)],
        ])

        # 2. Prediction Error
        error = self.k * cur - (phi.T @ self.theta)[0, 0]

        # 3. Calculate Gain Vector (K)
        # K tells us how much to change parameters based on the error
        num = self.P @ phi
        den = self.lmbda + (phi.T @ self.P @ phi)
        K = num / den

        # 4. Update Estimates
        new_theta = self.theta + K * error
        self.theta = np.clip(new_theta, self.theta_bounds[:, [0]], self.theta_bounds[:, [1]])

        # 5. Update Covariance Matrix (P)
        self.P = (self.P - (K @ phi.T @ self.P)) / self.lmbda

    def predict(self, pos: float) -> float:
        """
        Use learned parameters to make a prediction of current.

        Parameters:
        - pos (float): Absolute angular position in degrees

        Returns:
        - (float): Current prediction in amperes
        """

        pos = (pos % 360) * np.pi / 180     # Convert to radians
        phi = np.array([
            [np.cos(pos)],
        ])

        return (phi.T @ self.theta)[0, 0]

