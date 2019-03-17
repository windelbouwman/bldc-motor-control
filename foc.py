""" Field oriented control.

"""

import numpy as np


class FocControl:
    """ Field oriented control.

    Note: This is not an efficient algorithm, but intended
    for simplicity and readability.
    """
    def __init__(self, ts):
        self.ts = ts
        self.d_setpoint = 0
        self.q_setpoint = 5
        self.d_actual = 0
        self.q_actual = 0
        self.Kp = 0.05
        self.Ki = 0.05
        self.d_error = 0
        self.d_error_int = 0
        self.q_error = 0
        self.q_error_int = 0

    def do_control(self, i_a, i_b, i_c, theta):
        # One of the clarke transforms:
        K_C = np.sqrt(2/3)*np.array([
            [1, -1/2, -1/2],
            [0, np.sqrt(3)/2, -np.sqrt(3)/2],
            [1/np.sqrt(2), 1/np.sqrt(2), 1/np.sqrt(2)],
        ])
        i = np.array([i_a, i_b, i_c])
        alpha_beta_gamma = K_C @ i
        # print(alpha_beta_gamma)

        # Park:
        K_P = np.array([
            [np.cos(theta), np.sin(theta), 0],
            [-np.sin(theta), np.cos(theta), 0],
            [0, 0, 1],
        ])

        x = K_P @ alpha_beta_gamma
        self.d_actual = x[0]
        self.q_actual = x[1]

        self.d_error = self.d_setpoint - self.d_actual
        self.d_error_int += self.d_error
        self.q_error = self.q_setpoint - self.q_actual
        self.q_error_int += self.q_error

        self.d_output = self.d_error * self.Kp + self.d_error_int * self.Ki
        self.q_output = self.q_error * self.Kp + self.q_error_int * self.Ki

        # Inverse park, then inverse clarke:
        K_P_inv = np.linalg.inv(K_P)
        K_C_inv = np.linalg.inv(K_C)

        x = np.array([self.d_output, self.q_output, 0])
        x = K_C_inv @ K_P_inv @ x
        v_a = x[0]
        v_b = x[1]
        v_c = x[2]

        return [v_a, v_b, v_c]

