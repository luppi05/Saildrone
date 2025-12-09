import numpy as np
import matplotlib.pyplot as plt

def step_euler(state_deriv, t, dt, z):

    return z + dt * state_deriv(t, z)


def step_rk(state_deriv, t, dt, z):

    k1 = state_deriv(t, z, log=True)
    k2 = state_deriv(t + 0.5*dt, z + 0.5*dt*k1, log=False)
    k3 = state_deriv(t + 0.5*dt, z + 0.5*dt*k2, log=False)
    k4 = state_deriv(t + dt,     z + dt*k3, log=False)

    return z + (dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)

#solve initial value problem
def solve_ivp(state_deriv, t0, tmax, dt, z0, method='RK'):

    z = z0.reshape(-1, 1)
    t = np.array([t0])

    while t[-1] <= tmax:
        t_current = t[-1]

        if method == 'Euler':
            z_next = step_euler(state_deriv, t_current, dt, z[:, -1])
        else:
            z_next = step_rk(state_deriv, t_current, dt, z[:, -1])


        z = np.append(z, z_next.reshape(-1, 1), axis=1)
        t = np.append(t, t_current + dt)

    return t, z
