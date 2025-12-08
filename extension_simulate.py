import numpy as np
import matplotlib.pyplot as plt
from ode import solve_ivp
from extension_programme import dynamics_auto
from extension_programme import waypoints


#simulate from 0s to 600s with initial conditions
def simulate_auto():

    t0 = 0
    tmax = 600
    dt = 0.05

    z0 = np.array([
        0.0,
        0.0,
        np.pi/2,
        0.0,
        2.9,
        0.0
    ])
#solve ODE
    t, z = solve_ivp(dynamics_auto, t0, tmax, dt, z0, method='RK')
    return t, z

#plots trajectory of drone
def plot_trajectory(t, z):
    x = z[0]
    y = z[1] 

    plt.figure(figsize=(8, 6))
    plt.plot(x, y, 'b', linewidth=2)
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.axis("equal")
    plt.grid(True)
    plt.show()

    wps = np.array(waypoints)
    plt.scatter(wps[:,0], wps[:,1], c='red', s=100, marker='o')


if __name__ == "__main__":
    t, z = simulate_auto()
    plot_trajectory(t, z)
