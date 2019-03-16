
import math
from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt


class BldcMotor:
    """ Simulated model of 3-phase BLDC motor.
    """
    def __init__(self):
        # Parameters:
        self.P = 2  # Poles
        self.L = 0.143e-3  # phase inductance (H)
        self.R = 1.27  # phase resistance (ohm)
        self.beta = 0.0001  # Mechanical friction
        self.J = 0.000001  # Mechanical rotational inertia
        self.k_e = 0.01  # back emf constant
        self.k_t = 0.01  # Motor constant

        # Phase locations:
        self.phase_a = 0
        self.phase_b = -(math.pi * 2)/3
        self.phase_c = -(math.pi * 4)/3

        # Inputs:
        self.v_a = 0
        self.v_b = 0
        self.v_c = 0

        # System states:
        self.i_a = 0
        self.i_b = 0
        self.omega_m = 0  # mechanical speed
        self.theta_m = 0  # mechanical angle

        self.i_c = 0  # Calculated from i_a and i_b

    def set_voltages(self, v_a, v_b, v_c):
        self.v_a = v_a
        self.v_b = v_b
        self.v_c = v_c

    @staticmethod
    def F(angle):
        # Plugin here trapeziodal or sine function:
        return math.sin(angle)

    def bldc(self):
        """ Calculate state derivatives.
        """

        # Calculate derived symbols:
        self.i_c = -(self.i_a + self.i_b)

        # Electrical angle depends on number of poles:
        theta_e = self.theta_m * (self.P / 2)

        # Forward torque from all phase currents:
        T_a = self.k_t * self.i_a * self.F(theta_e + self.phase_a)
        T_b = self.k_t * self.i_b * self.F(theta_e + self.phase_b)
        T_c = self.k_t * self.i_c * self.F(theta_e + self.phase_c)
        T_e = T_a + T_b + T_c

        # back emf:
        e_a = self.k_e * self.omega_m * self.F(theta_e + self.phase_a)
        e_b = self.k_e * self.omega_m * self.F(theta_e + self.phase_b)
        e_c = self.k_e * self.omega_m * self.F(theta_e + self.phase_c)

        # derivatives:
        di_a = (-3 * self.R * self.i_a - 2 * e_a + e_b + e_c + 2 * self.v_a - self.v_b - self.v_c) / (3 * self.L)
        di_b = (-3 * self.R * self.i_b + e_a - 2 * e_b + e_c - self.v_a + 2 * self.v_b - self.v_c) / (3 * self.L)
        dtheta_m = self.omega_m
        domega_m = -(self.beta/self.J) * self.omega_m + (1/self.J) * T_e
        dy = [di_a, di_b, domega_m, dtheta_m]
        return dy

    def get_state(self):
        """ Get current state vector.
        """
        return [self.i_a, self.i_b, self.omega_m, self.theta_m]
    
    def set_state(self, state):
        """ Set current state from vector.

        Mainly used to be able to simulate with different ODE solvers.
        """
        self.i_a = state[0]
        self.i_b = state[1]
        self.omega_m = state[2]
        self.theta_m = state[3]

    def odefunc(self, y, t):
        """ Pass this to function scipy.integrate.odeint """
        self.set_state(y)
        return self.bldc()


class BldcControlStrategy1:
    """ Simple forced sine wave.

    This is a stateless control.
    """
    def do_control(self, t):
        # Generate a sine wave:
        control_freq = 2
        control_phase = 2 * math.pi * control_freq * t
        v_a = 15*math.sin(control_phase)
        v_b = 15*math.sin(control_phase - 2*math.pi/3)
        v_c = 15*math.sin(control_phase - 4*math.pi/3)
        return v_a, v_b, v_c


def main():
    strategy1()


def strategy1():
    """ Try-out control strategy 1. """
    motor = BldcMotor()
    control = BldcControlStrategy1()
    t = np.linspace(0, 10, 10000)
    y0 = motor.get_state()
    def f(y, t):
        motor.set_state(y)
        voltages = control.do_control(t)
        motor.set_voltages(*voltages)
        return motor.bldc()

    sol = odeint(f, y0, t)

    plt.plot(t, sol)
    plt.grid()
    plt.show()


main()