import numpy as np
import matplotlib.pyplot as plt

from ode import solve_ivp
from programme import dynamics
#simulate
def simulate():

    t0 = 0
    tmax = 120
    dt = 0.05
#initial state
    z0 = np.array([
        0.0,
        0.0,
        np.pi/2,
        0.0,
        2.9,
        0.0         
    ])

    t, z = solve_ivp(dynamics, t0, tmax, dt, z0, method='RK')
    return t, z

#plots trajectory of sail drone
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

if __name__ == "__main__":
    t, z = simulate()
    plot_trajectory(t, z)
