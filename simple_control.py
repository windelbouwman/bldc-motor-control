import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

from bldc import BldcMotor


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
    plt.legend()
    plt.grid()
    plt.show()


if __name__ == '__main__':
    strategy1()
