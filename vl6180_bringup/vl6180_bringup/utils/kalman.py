#!/usr/bin/env python3
"""
Kalman filter
Luke Strohbehn, https://arxiv.org/pdf/1204.0375.pdf
"""

import numpy as np
from filterpy.kalman import KalmanFilter


class Kalman:
    """
    X: The mean state estimate of the previous step ( k-1 ).
    P : The state covariance of previous step ( k-1 ).
    A : The transition n×n matrix.
    Q : The process noise covariance matrix.
    B : The input effect matrix.
    U : The control input.
    """

    def __init__(self) -> None:
        self.X = np.zeros(2)
        self.P = np.zeros(2)
        self.A = np.zeros((2, 2))
        self.Q = np.zeros(2)
        self.B = np.zeros(2)
        self.U = np.zeros(2)
        return

    def predict(self):
        self.X = np.dot(self.A, self.X) + np.dot(self.B, self.U)
        self.P = np.dot(self.A, np.dot(self.P, self.A.T)) + self.Q
        return

    def update(self):
        return


def main():
    f = KalmanFilter(dim_x=3, dim_z=1)
    f.x = np.array([30])
    f.F = np.identity(3)
    f.H = np.array([1])
    f.P = np.identity(f.dim_x) * 5
    f.R = 5
    return


if __name__ == "__main__":
    main()
