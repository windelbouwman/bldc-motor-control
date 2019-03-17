""" Mixed discrete continuous time simulation.

"""

import matplotlib.pyplot as plt
import numpy as np
import tqdm

from bldc import BldcMotor
from foc import FocControl


def discrete_control():
    motor = BldcMotor()
    t_end = 2
    t = 0
    time = []
    logs = []
    tick_rate = 1000  # [Hz]
    ticks = range(t_end * tick_rate)
    ts = 1 / tick_rate  # sample rate
    control = FocControl(ts)
    for _ in tqdm.tqdm(ticks, unit='s', unit_scale=ts):
        # Logging:
        time.append(t)
        log = (
            motor.i_a, motor.i_b, motor.i_c, motor.theta_m,
            control.d_error, control.q_error,
            control.d_setpoint, control.q_setpoint,
            control.d_actual, control.q_actual,
            motor.v_a, motor.v_b, motor.v_c, motor.omega_m,
        )
        logs.append(log)

        # print(tick)
        # Simulate control and motor in lock-step
        # First do some control:
        voltages = control.do_control(motor.i_a, motor.i_b, motor.i_c, motor.theta_m)

        # Now do motor:
        motor.set_voltages(*voltages)
        motor.proceed(ts)

        t += ts
    
    # Plotting:
    time = np.array(time)
    logs = np.array(logs)
    print('Number of plot points:', logs.shape)

    # Phase currents:
    plt.subplot(3, 3, 1)
    plt.plot(time, logs[:, 0], label='i_a')
    plt.plot(time, logs[:, 1], label='i_b')
    plt.plot(time, logs[:, 2], label='i_c')
    plt.xlabel('time [s]')
    plt.ylabel('Current [A]')
    plt.grid()
    plt.legend()

    plt.subplot(3, 3, 2)
    plt.plot(time, logs[:, 10], label='v_a')
    plt.plot(time, logs[:, 11], label='v_b')
    plt.plot(time, logs[:, 12], label='v_c')
    plt.xlabel('time [s]')
    plt.ylabel('voltage [V]')
    plt.grid()
    plt.legend()

    plt.subplot(3, 3, 3)
    plt.plot(time, logs[:, 3], label='theta')
    plt.xlabel('time [s]')
    plt.grid()
    plt.legend()

    plt.subplot(3, 3, 4)
    plt.plot(time, logs[:, 13], label='omega')
    plt.xlabel('time [s]')
    plt.grid()
    plt.legend()

    plt.subplot(3, 3, 5)
    plt.plot(time, logs[:, 4], label='d_error')
    plt.xlabel('time [s]')
    plt.grid()
    plt.legend()

    plt.subplot(3, 3, 6)
    plt.plot(time, logs[:, 7], label='q_actual')
    plt.plot(time, logs[:, 9], label='q_setpoint')
    plt.xlabel('time [s]')
    plt.grid()
    plt.legend()

    plt.subplot(3, 3, 7)
    plt.plot(time, logs[:, 6], label='d_actual')
    plt.plot(time, logs[:, 8], label='d_setpoint')
    plt.xlabel('time [s]')
    plt.grid()
    plt.legend()

    plt.subplot(3, 3, 8)
    plt.plot(time, logs[:, 5], label='q_error')
    plt.xlabel('time [s]')
    plt.grid()
    plt.legend()

    plt.tight_layout()
    plt.show()



if __name__ == '__main__':
    discrete_control()
